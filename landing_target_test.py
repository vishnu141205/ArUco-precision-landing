#!/usr/bin/env python3
"""
BENCH TEST ONLY — ArUco -> MAVLink LANDING_TARGET for PX4.

This script does NOT arm, change flight mode, or command motors.
Keep the vehicle disarmed and remove propellers.

Setup:
- Pixhawk 6X / PX4
- Raspberry Pi -> TELEM2 via /dev/serial0 @ 57600
- Downward Pi camera
- image top = aircraft front
- image right = aircraft right
- ArUco DICT_4X4_50, ID 0, outer side = 0.168 m
- camera_calibration.npz created at 640x480
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
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok:
        return None
    return tvec.reshape(3)


def send_landing_target(vehicle, target_ned, distance_m):
    target_type = getattr(
        mavutil.mavlink,
        "LANDING_TARGET_TYPE_VISION_FIDUCIAL",
        2,
    )

    msg = mavutil.mavlink.MAVLink_landing_target_message(
        time_usec=int(time.time() * 1_000_000),
        target_num=0,
        frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        angle_x=0.0,
        angle_y=0.0,
        distance=float(distance_m),
        size_x=0.0,
        size_y=0.0,
        x=float(target_ned[0]),
        y=float(target_ned[1]),
        z=float(target_ned[2]),
        q=[1.0, 0.0, 0.0, 0.0],
        type=target_type,
        position_valid=1,
    )
    vehicle.mav.send(msg)


def main():
    calibration_path = Path(CALIBRATION_FILE)
    if not calibration_path.exists():
        raise FileNotFoundError(f"{CALIBRATION_FILE} not found")

    calibration = np.load(calibration_path)
    camera_matrix = calibration["camera_matrix"].astype(np.float64)
    dist_coeffs = calibration["dist_coeffs"].astype(np.float64)

    print(f"Connecting to Pixhawk on {PORT} @ {BAUD}...")
    vehicle = mavutil.mavlink_connection(
        PORT,
        baud=BAUD,
        autoreconnect=True,
    )
    hb = vehicle.wait_heartbeat(timeout=15)
    if hb is None:
        raise RuntimeError("No Pixhawk heartbeat received")

    print("Pixhawk connected.")
    print("BENCH TEST ONLY: no arming or mode changes are sent.\n")

    camera = Picamera2()
    config = camera.create_video_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
        controls={"FrameRate": 30},
    )
    camera.configure(config)
    camera.start()
    time.sleep(2.0)

    detector = make_detector()

    roll = pitch = yaw = None
    local_position = None
    filtered_body = None

    send_period = 1.0 / SEND_RATE_HZ
    print_period = 1.0 / PRINT_RATE_HZ
    last_send = 0.0
    last_print = 0.0
    sent_count = 0

    try:
        while True:
            # Drain MAVLink telemetry.
            while True:
                msg = vehicle.recv_match(blocking=False)
                if msg is None:
                    break

                if msg.get_type() == "ATTITUDE":
                    roll = float(msg.roll)
                    pitch = float(msg.pitch)
                    yaw = float(msg.yaw)
                elif msg.get_type() == "LOCAL_POSITION_NED":
                    local_position = np.array(
                        [float(msg.x), float(msg.y), float(msg.z)],
                        dtype=np.float64,
                    )

            frame = camera.capture_array()
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            corners_list, ids, _ = detector.detectMarkers(gray)

            target_seen = False

            if ids is not None and roll is not None and local_position is not None:
                for marker_corners, marker_id in zip(corners_list, ids.flatten()):
                    if int(marker_id) != TARGET_ID:
                        continue

                    tvec = estimate_pose(marker_corners, camera_matrix, dist_coeffs)
                    if tvec is None or tvec[2] <= 0:
                        continue

                    distance_m = float(np.linalg.norm(tvec))

                    # Camera -> BODY_FRD; this mounting has already been bench verified.
                    camera_x, camera_y, camera_z = map(float, tvec)
                    body = np.array([
                        -camera_y,   # forward
                        +camera_x,   # right
                        +camera_z,   # down
                    ], dtype=np.float64)

                    if filtered_body is None:
                        filtered_body = body.copy()
                    else:
                        filtered_body = (
                            FILTER_ALPHA * body
                            + (1.0 - FILTER_ALPHA) * filtered_body
                        )

                    relative_ned = body_to_ned_rotation(roll, pitch, yaw) @ filtered_body
                    target_ned = local_position + relative_ned
                    target_seen = True

                    now = time.monotonic()
                    if now - last_send >= send_period:
                        send_landing_target(vehicle, target_ned, distance_m)
                        last_send = now
                        sent_count += 1

                    if now - last_print >= print_period:
                        print(
                            f"BODY   | F={filtered_body[0]:+.3f} "
                            f"R={filtered_body[1]:+.3f} D={filtered_body[2]:+.3f}"
                        )
                        print(
                            f"REL NED| N={relative_ned[0]:+.3f} "
                            f"E={relative_ned[1]:+.3f} D={relative_ned[2]:+.3f}"
                        )
                        print(
                            f"VEH NED| N={local_position[0]:+.3f} "
                            f"E={local_position[1]:+.3f} D={local_position[2]:+.3f}"
                        )
                        print(
                            f"TARGET | N={target_ned[0]:+.3f} "
                            f"E={target_ned[1]:+.3f} D={target_ned[2]:+.3f}"
                        )
                        print(f"LANDING_TARGET sent: {sent_count}\n")
                        last_print = now

                    break

            if not target_seen:
                filtered_body = None
                now = time.monotonic()
                if now - last_print >= print_period:
                    if roll is None:
                        print("Waiting for PX4 ATTITUDE...")
                    elif local_position is None:
                        print("Waiting for LOCAL_POSITION_NED...")
                    else:
                        print("Searching for ArUco ID 0...")
                    last_print = now

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        camera.stop()


if __name__ == "__main__":
    main()
