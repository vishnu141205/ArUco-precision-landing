#!/usr/bin/env python3
"""
ArUco auto-land for PX4 (Pixhawk 6X + Raspberry Pi on TELEM2).

Behaviour
---------
1. The pilot flies the drone manually with the RC transmitter. This script
   never arms, never takes off and never touches the sticks / flight mode
   while the marker has not been confirmed.
2. The whole time, it streams MAVLink LANDING_TARGET messages when the marker
   (DICT_4X4_50, ID 0) is visible, so PX4's precision-land logic has data.
3. When the marker has been seen for CONFIRM_FRAMES consecutive frames while the
   vehicle is armed, in the air and within ENGAGE_MAX_DISTANCE_M, the script
   switches PX4 to AUTO.PRECLAND (falls back to AUTO.LAND if that is not
   accepted). PX4 then flies over the marker and lands by itself and disarms.
4. LOCK_OUT_STICKS: during the autonomous landing PX4's stick override
   (COM_RC_OVERRIDE) is turned off, so pilot stick movements do NOT interrupt
   the landing. The original value is restored afterwards.

Pilot safety net (kept on purpose)
----------------------------------
Stick input is ignored during the landing, but the RC *flight-mode switch*
and the kill switch still work. If the pilot flips the mode switch, the
script accepts that, stops commanding, and will not re-engage until the
vehicle has landed / disarmed. Do not remove this.

Required PX4 parameters (set once in QGroundControl)
----------------------------------------------------
  PLD_BTOUT, PLD_HACC_RAD, PLD_FAPPR_ALT, PLD_MAX_SRCH  (precision landing)
  LTEST_MODE  (landing target estimator, 1 = moving / 0 = stationary)
  COM_RC_OVERRIDE  (script manages it if LOCK_OUT_STICKS is True)
TEST FIRST WITH PROPELLERS OFF, THEN TETHERED / OPEN FIELD.
"""

import os
os.environ.setdefault("MAVLINK20", "1")

import math
import time
from pathlib import Path

import cv2
import numpy as np
from picamera2 import Picamera2
from pymavlink import mavutil

# ---------------- configuration ----------------
PORT = "/dev/serial0"
BAUD = 57600

TARGET_ID = 0
MARKER_SIZE_M = 0.168
ARUCO_DICT_ID = cv2.aruco.DICT_4X4_50

WIDTH = 640
HEIGHT = 480
CALIBRATION_FILE = "camera_calibration.npz"

SEND_RATE_HZ = 15.0
PRINT_RATE_HZ = 2.0
FILTER_ALPHA = 0.35

CONFIRM_FRAMES = 15            # consecutive detections before engaging
ENGAGE_MAX_DISTANCE_M = 15.0   # ignore a marker farther than this
MIN_AIRBORNE_ALT_M = 1.0       # must be this high (NED z <= -value) to engage
LOCK_OUT_STICKS = True         # ignore pilot sticks once landing has started
ACK_TIMEOUT_S = 2.0

# PX4 custom mode encoding
PX4_MAIN_MODE_AUTO = 4
PX4_SUB_AUTO_LAND = 6
PX4_SUB_AUTO_PRECLAND = 9


def body_to_ned_rotation(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ], dtype=np.float64)


def make_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(dictionary, params)


def estimate_pose(marker_corners, camera_matrix, dist_coeffs):
    half = MARKER_SIZE_M / 2.0
    object_points = np.array([
        [-half, +half, 0.0],
        [+half, +half, 0.0],
        [+half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float64)
    image_points = marker_corners.reshape(4, 2).astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(
        object_points, image_points, camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    return tvec.reshape(3) if ok else None


def send_landing_target(vehicle, target_ned, distance_m):
    target_type = getattr(
        mavutil.mavlink, "LANDING_TARGET_TYPE_VISION_FIDUCIAL", 2)
    vehicle.mav.send(mavutil.mavlink.MAVLink_landing_target_message(
        time_usec=int(time.time() * 1_000_000),
        target_num=0,
        frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        angle_x=0.0, angle_y=0.0,
        distance=float(distance_m),
        size_x=0.0, size_y=0.0,
        x=float(target_ned[0]), y=float(target_ned[1]), z=float(target_ned[2]),
        q=[1.0, 0.0, 0.0, 0.0],
        type=target_type,
        position_valid=1,
    ))


# ---------------- PX4 helpers ----------------
def set_px4_mode(vehicle, main_mode, sub_mode):
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        main_mode, sub_mode, 0, 0, 0, 0,
    )


def decode_mode(custom_mode):
    return (custom_mode >> 16) & 0xFF, (custom_mode >> 24) & 0xFF


def get_param(vehicle, name, timeout=3.0):
    vehicle.mav.param_request_read_send(
        vehicle.target_system, vehicle.target_component,
        name.encode(), -1)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        m = vehicle.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if m is not None and m.param_id.rstrip("\x00") == name:
            return m.param_value
    return None


def set_param_int(vehicle, name, value):
    vehicle.mav.param_set_send(
        vehicle.target_system, vehicle.target_component,
        name.encode(), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32)


class LandingState:
    IDLE = "IDLE"              # pilot flying, watching for marker
    LANDING = "LANDING"        # we commanded PRECLAND/LAND
    PILOT_TOOK_OVER = "PILOT"  # pilot used mode switch; hands off until disarm


def main():
    cal_path = Path(CALIBRATION_FILE)
    if not cal_path.exists():
        raise FileNotFoundError(f"{CALIBRATION_FILE} not found")
    cal = np.load(cal_path)
    camera_matrix = cal["camera_matrix"].astype(np.float64)
    dist_coeffs = cal["dist_coeffs"].astype(np.float64)

    print(f"Connecting to Pixhawk on {PORT} @ {BAUD}...")
    vehicle = mavutil.mavlink_connection(PORT, baud=BAUD, autoreconnect=True)
    if vehicle.wait_heartbeat(timeout=15) is None:
        raise RuntimeError("No Pixhawk heartbeat received")
    print("Pixhawk connected. Pilot flies; auto-land engages on marker.\n")

    camera = Picamera2()
    camera.configure(camera.create_video_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
        controls={"FrameRate": 30},
    ))
    camera.start()
    time.sleep(2.0)
    detector = make_detector()

    roll = pitch = yaw = None
    local_position = None
    armed = False
    custom_mode = 0
    filtered_body = None
    seen_frames = 0

    state = LandingState.IDLE
    engaged_at = 0.0
    engaged_modes = ()
    saved_rc_override = None

    send_period = 1.0 / SEND_RATE_HZ
    print_period = 1.0 / PRINT_RATE_HZ
    last_send = last_print = 0.0

    def restore_stick_override():
        nonlocal saved_rc_override
        if saved_rc_override is not None:
            set_param_int(vehicle, "COM_RC_OVERRIDE", int(saved_rc_override))
            print(f"COM_RC_OVERRIDE restored to {int(saved_rc_override)}")
            saved_rc_override = None

    try:
        while True:
            # ---- telemetry ----
            while True:
                msg = vehicle.recv_match(blocking=False)
                if msg is None:
                    break
                t = msg.get_type()
                if t == "ATTITUDE":
                    roll, pitch, yaw = float(msg.roll), float(msg.pitch), float(msg.yaw)
                elif t == "LOCAL_POSITION_NED":
                    local_position = np.array(
                        [msg.x, msg.y, msg.z], dtype=np.float64)
                elif t == "HEARTBEAT" and msg.get_srcComponent() == 1:
                    armed = bool(msg.base_mode
                                 & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    custom_mode = msg.custom_mode

            # ---- state housekeeping ----
            if not armed and state != LandingState.IDLE:
                print("Disarmed - landing complete / reset.")
                restore_stick_override()
                state = LandingState.IDLE
                seen_frames = 0

            if state == LandingState.LANDING:
                if decode_mode(custom_mode) not in engaged_modes and \
                        time.monotonic() - engaged_at > ACK_TIMEOUT_S:
                    # Mode changed away from land/precland -> pilot used switch.
                    print("Flight mode changed by pilot - hands off.")
                    restore_stick_override()
                    state = LandingState.PILOT_TOOK_OVER

            # ---- vision ----
            frame = camera.capture_array()
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            corners_list, ids, _ = detector.detectMarkers(gray)

            target_seen = False
            distance_m = None

            if ids is not None and roll is not None and local_position is not None:
                for mc, mid in zip(corners_list, ids.flatten()):
                    if int(mid) != TARGET_ID:
                        continue
                    tvec = estimate_pose(mc, camera_matrix, dist_coeffs)
                    if tvec is None or tvec[2] <= 0:
                        continue

                    distance_m = float(np.linalg.norm(tvec))
                    cx, cy_, cz = map(float, tvec)
                    body = np.array([-cy_, +cx, +cz], dtype=np.float64)

                    filtered_body = body.copy() if filtered_body is None else (
                        FILTER_ALPHA * body + (1.0 - FILTER_ALPHA) * filtered_body)

                    rel_ned = body_to_ned_rotation(roll, pitch, yaw) @ filtered_body
                    target_ned = local_position + rel_ned
                    target_seen = True

                    now = time.monotonic()
                    if now - last_send >= send_period:
                        send_landing_target(vehicle, target_ned, distance_m)
                        last_send = now
                    break

            if target_seen:
                seen_frames += 1
            else:
                seen_frames = 0
                filtered_body = None

            # ---- engage autonomous landing ----
            airborne = (local_position is not None
                        and local_position[2] <= -MIN_AIRBORNE_ALT_M)
            if (state == LandingState.IDLE and armed and airborne
                    and target_seen
                    and seen_frames >= CONFIRM_FRAMES
                    and distance_m <= ENGAGE_MAX_DISTANCE_M):
                print(f"Marker confirmed at {distance_m:.1f} m - AUTO LAND ENGAGED")

                if LOCK_OUT_STICKS:
                    saved_rc_override = get_param(vehicle, "COM_RC_OVERRIDE")
                    if saved_rc_override is not None:
                        set_param_int(vehicle, "COM_RC_OVERRIDE", 0)
                    else:
                        print("WARN: could not read COM_RC_OVERRIDE; "
                              "sticks may still interrupt the landing.")

                set_px4_mode(vehicle, PX4_MAIN_MODE_AUTO, PX4_SUB_AUTO_PRECLAND)
                engaged_modes = ((PX4_MAIN_MODE_AUTO, PX4_SUB_AUTO_PRECLAND),)
                engaged_at = time.monotonic()
                state = LandingState.LANDING

                # Fallback: if PRECLAND was not accepted, plain AUTO.LAND.
                deadline = time.monotonic() + ACK_TIMEOUT_S
                while time.monotonic() < deadline:
                    m = vehicle.recv_match(type="HEARTBEAT", blocking=True,
                                           timeout=0.3)
                    if m is not None and m.get_srcComponent() == 1:
                        custom_mode = m.custom_mode
                        if decode_mode(custom_mode) == engaged_modes[0]:
                            break
                else:
                    print("PRECLAND not accepted - falling back to AUTO.LAND")
                    set_px4_mode(vehicle, PX4_MAIN_MODE_AUTO, PX4_SUB_AUTO_LAND)
                    engaged_modes = ((PX4_MAIN_MODE_AUTO, PX4_SUB_AUTO_LAND),
                                     (PX4_MAIN_MODE_AUTO, PX4_SUB_AUTO_PRECLAND))
                    engaged_at = time.monotonic()

            # ---- status print ----
            now = time.monotonic()
            if now - last_print >= print_period:
                if roll is None or local_position is None:
                    print("Waiting for ATTITUDE / LOCAL_POSITION_NED...")
                else:
                    seen = f"{distance_m:.2f} m" if target_seen else "no marker"
                    print(f"[{state}] armed={armed} alt="
                          f"{-local_position[2]:.1f} m  {seen}  "
                          f"seen={seen_frames}")
                last_print = now

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        restore_stick_override()
        camera.stop()


if __name__ == "__main__":
    main()
