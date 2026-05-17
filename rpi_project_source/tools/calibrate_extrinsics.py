"""
Calibration tool for Camera <-> LiDAR extrinsics.
Uses OpenCV chessboard detection to find camera pose relative to a physical target,
and matches it with LiDAR points if possible (or manual measurement).
"""

import argparse
import json

import cv2
import numpy as np


def calibrate_extrinsics(
    image_path: str, square_size_mm: float = 25.0, pattern_size: tuple = (9, 6)
):
    """
    Very simplified extrinsic calibration script.
    In real usage, you'd collect multiple frames.
    """
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: Could not load image {image_path}")
        return

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

    if ret:
        print(f"Chessboard found in {image_path}")

        # 3D points of chessboard in its own frame
        objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        objp[:, :2] = (
            np.mgrid[0 : pattern_size[0], 0 : pattern_size[1]].T.reshape(-1, 2)
            * square_size_mm
        )

        # Assuming we have intrinsics (placeholder)
        # In real life, load from config.json
        h, w = img.shape[:2]
        mtx = np.array([[w, 0, w / 2], [0, w, h / 2], [0, 0, 1]], dtype=np.float32)
        dist = np.zeros(5)

        # Find pose of chessboard relative to camera
        _, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist)

        # tvec is [x, y, z] in mm
        print(f"Camera Pose relative to target: T={tvec.flatten()}, R={rvec.flatten()}")

        # Save placeholder extrinsics to config snippet
        extrinsics = {
            "camera_extrinsics": {
                "x": 0.1,  # Distance from center of rear axle forward (m)
                "y": 0.0,  # Lateral offset (m)
                "z": 0.15,  # Height from ground (m)
                "pitch": 0.0,  # Tilt angle (rad)
            }
        }

        print("\nSuggested config.json update:")
        print(json.dumps(extrinsics, indent=4))

        # Draw and show
        cv2.drawChessboardCorners(img, pattern_size, corners, ret)
        cv2.imshow("Calibration", img)
        cv2.waitKey(2000)
    else:
        print("Chessboard NOT found. Ensure target is visible and well-lit.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=str, help="Path to calibration image")
    args = parser.parse_args()

    if args.image:
        calibrate_extrinsics(args.image)
    else:
        print("Usage: python calibrate_extrinsics.py --image path/to/image.jpg")
