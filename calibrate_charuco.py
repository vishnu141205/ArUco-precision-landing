#!/usr/bin/env python3
"""
calibrate_charuco.py

Camera calibration for this exact printed board:

    ChArUco board: 7 x 10 squares
    Dictionary:    DICT_5X5_100
    Square size:   24.0 mm = 0.024 m
    Marker size:   18.0 mm = 0.018 m
    Board size:    168 x 240 mm

Controls:
    SPACE  Capture current good view
    R      Remove last capture
    C      Calibrate and save camera_calibration.npz
    Q      Quit

Recommended:
    Capture 20-30 GOOD, VARIED views.
"""

import time
from pathlib import Path

import cv2
import numpy as np
from picamera2 import Picamera2


SQUARES_X = 7
SQUARES_Y = 10
SQUARE_LENGTH_M = 0.024
MARKER_LENGTH_M = 0.018
DICTIONARY_ID = cv2.aruco.DICT_5X5_100

WIDTH = 640
HEIGHT = 480

OUTPUT_FILE = "camera_calibration.npz"
MIN_CAPTURED_VIEWS = 15
MIN_CHARUCO_CORNERS = 12


def build_board():
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY_ID)
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        SQUARE_LENGTH_M,
        MARKER_LENGTH_M,
        dictionary,
    )
    return dictionary, board


def detect_charuco(gray, dictionary, board):
    if hasattr(cv2.aruco, "CharucoDetector"):
        detector = cv2.aruco.CharucoDetector(board)
        charuco_corners, charuco_ids, marker_corners, marker_ids = (
            detector.detectBoard(gray)
        )
        return charuco_corners, charuco_ids, marker_corners, marker_ids

    params = cv2.aruco.DetectorParameters()
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
        gray,
        dictionary,
        parameters=params,
    )

    if marker_ids is None or len(marker_ids) == 0:
        return None, None, marker_corners, marker_ids

    count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners,
        marker_ids,
        gray,
        board,
    )

    if count is None or count == 0:
        return None, None, marker_corners, marker_ids

    return charuco_corners, charuco_ids, marker_corners, marker_ids


def main():
    dictionary, board = build_board()

    captured_corners = []
    captured_ids = []

    print()
    print("==========================================")
    print(" ChArUco CAMERA CALIBRATION")
    print("==========================================")
    print(f"Board             : {SQUARES_X} x {SQUARES_Y} squares")
    print("Dictionary        : DICT_5X5_100")
    print(f"Square size       : {SQUARE_LENGTH_M * 1000:.1f} mm")
    print(f"Marker size       : {MARKER_LENGTH_M * 1000:.1f} mm")
    print(f"Resolution        : {WIDTH} x {HEIGHT}")
    print()
    print("SPACE = capture")
    print("R     = remove last capture")
    print("C     = calibrate and save")
    print("Q     = quit")
    print("==========================================")
    print()

    camera = Picamera2()
    config = camera.create_video_configuration(
        main={
            "size": (WIDTH, HEIGHT),
            "format": "RGB888",
        },
        controls={
            "FrameRate": 30,
        },
    )

    camera.configure(config)
    camera.start()
    time.sleep(2.0)

    image_size = (WIDTH, HEIGHT)

    try:
        while True:
            frame_rgb = camera.capture_array()

            frame_bgr = cv2.cvtColor(
                frame_rgb,
                cv2.COLOR_RGB2BGR,
            )

            gray = cv2.cvtColor(
                frame_bgr,
                cv2.COLOR_BGR2GRAY,
            )

            (
                charuco_corners,
                charuco_ids,
                marker_corners,
                marker_ids,
            ) = detect_charuco(
                gray,
                dictionary,
                board,
            )

            if marker_ids is not None and len(marker_ids) > 0:
                cv2.aruco.drawDetectedMarkers(
                    frame_bgr,
                    marker_corners,
                    marker_ids,
                )

            corner_count = 0

            if (
                charuco_corners is not None
                and charuco_ids is not None
            ):
                corner_count = len(charuco_ids)

                cv2.aruco.drawDetectedCornersCharuco(
                    frame_bgr,
                    charuco_corners,
                    charuco_ids,
                )

            good_frame = corner_count >= MIN_CHARUCO_CORNERS

            status = (
                f"ChArUco corners: {corner_count} | "
                f"Captured views: {len(captured_corners)}"
            )

            cv2.putText(
                frame_bgr,
                status,
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (0, 255, 0) if good_frame else (0, 0, 255),
                2,
            )

            cv2.putText(
                frame_bgr,
                "GOOD FRAME" if good_frame else "MOVE BOARD / IMPROVE VIEW",
                (15, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (0, 255, 0) if good_frame else (0, 0, 255),
                2,
            )

            cv2.putText(
                frame_bgr,
                "SPACE=capture  R=undo  C=calibrate  Q=quit",
                (15, HEIGHT - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1,
            )

            cv2.imshow(
                "ChArUco Camera Calibration",
                frame_bgr,
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord(" "):
                if not good_frame:
                    print(
                        f"Capture ignored: only {corner_count} corners "
                        f"(need at least {MIN_CHARUCO_CORNERS})."
                    )
                    continue

                captured_corners.append(
                    charuco_corners.copy()
                )

                captured_ids.append(
                    charuco_ids.copy()
                )

                print(
                    f"Captured view {len(captured_corners)} "
                    f"with {corner_count} ChArUco corners."
                )

                time.sleep(0.20)

            elif key in (ord("r"), ord("R")):
                if captured_corners:
                    captured_corners.pop()
                    captured_ids.pop()

                    print(
                        f"Removed last view. "
                        f"Remaining: {len(captured_corners)}"
                    )
                else:
                    print("No captures to remove.")

            elif key in (ord("c"), ord("C")):
                if len(captured_corners) < MIN_CAPTURED_VIEWS:
                    print(
                        f"Need at least {MIN_CAPTURED_VIEWS} views. "
                        f"Current: {len(captured_corners)}"
                    )
                    continue

                print()
                print("Calibrating...")

                (
                    rms,
                    camera_matrix,
                    dist_coeffs,
                    rvecs,
                    tvecs,
                ) = cv2.aruco.calibrateCameraCharuco(
                    charucoCorners=captured_corners,
                    charucoIds=captured_ids,
                    board=board,
                    imageSize=image_size,
                    cameraMatrix=None,
                    distCoeffs=None,
                )

                np.savez(
                    OUTPUT_FILE,
                    camera_matrix=camera_matrix,
                    dist_coeffs=dist_coeffs,
                    image_width=WIDTH,
                    image_height=HEIGHT,
                    rms=rms,
                    board_squares_x=SQUARES_X,
                    board_squares_y=SQUARES_Y,
                    board_square_length_m=SQUARE_LENGTH_M,
                    board_marker_length_m=MARKER_LENGTH_M,
                )

                print()
                print("==========================================")
                print(" CALIBRATION COMPLETE")
                print("==========================================")
                print(f"Saved: {Path(OUTPUT_FILE).resolve()}")
                print(f"RMS reprojection error: {rms:.6f} pixels")
                print()
                print("Camera matrix:")
                print(camera_matrix)
                print()
                print("Distortion coefficients:")
                print(dist_coeffs.ravel())
                print("==========================================")

                if rms < 0.5:
                    print("Calibration quality: EXCELLENT")
                elif rms < 1.0:
                    print("Calibration quality: GOOD")
                else:
                    print(
                        "Calibration quality: check board flatness, focus, "
                        "and capture more varied views."
                    )

                break

            elif key in (ord("q"), ord("Q")):
                print("Calibration cancelled.")
                break

    except KeyboardInterrupt:
        print("\nStopped.")

    finally:
        camera.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
