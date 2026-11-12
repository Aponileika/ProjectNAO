#!/usr/bin/env python3

from pathlib import Path

import cv2
import numpy as np


# Checkerboard settings. These are INNER corner counts, not square counts.
BOARD_COLUMNS = 7
BOARD_ROWS = 5

# Set this to the measured width of one checkerboard square. The translation
# vector and baseline will use the same unit (for example, 0.025 for metres).
SQUARE_SIZE = 0.029 

# The receiver creates calibration_images/left and calibration_images/right.
SCRIPT_DIRECTORY = Path(__file__).resolve().parent
IMAGE_DIRECTORY = SCRIPT_DIRECTORY / "calibration_images"
OUTPUT_FILE = SCRIPT_DIRECTORY / "stereo_calibration.txt"


def find_corners(image: np.ndarray):
    pattern_size = (BOARD_COLUMNS, BOARD_ROWS)
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
    found, corners = cv2.findChessboardCornersSB(image, pattern_size, flags)
    return corners if found else None


def matrix_text(name: str, matrix: np.ndarray) -> str:
    value = np.asarray(matrix)
    return f"{name} =\n{np.array2string(value, precision=12, suppress_small=False)}\n"


def main() -> None:
    # OpenCL does not benefit this calibration path and broken cached OpenCL
    # binaries on macOS can make corner detection appear to hang.
    cv2.ocl.setUseOpenCL(False)

    left_directory = IMAGE_DIRECTORY / "left"
    right_directory = IMAGE_DIRECTORY / "right"

    left_files = {path.name: path for path in left_directory.glob("*.png")}
    right_files = {path.name: path for path in right_directory.glob("*.png")}
    paired_names = sorted(left_files.keys() & right_files.keys())

    if not paired_names:
        raise RuntimeError(
            f"No matching PNG pairs found in {left_directory} and {right_directory}"
        )

    object_template = np.zeros(
        (BOARD_ROWS * BOARD_COLUMNS, 3), dtype=np.float32
    )
    object_template[:, :2] = (
        np.mgrid[0:BOARD_COLUMNS, 0:BOARD_ROWS].T.reshape(-1, 2)
        * SQUARE_SIZE
    )

    object_points = []
    left_points = []
    right_points = []
    accepted_names = []
    rejected_names = []
    reversed_right_names = []
    image_size = None

    print(f"Checking {len(paired_names)} matching image pairs...", flush=True)
    for pair_index, name in enumerate(paired_names, start=1):
        if pair_index == 1 or pair_index % 10 == 0 or pair_index == len(paired_names):
            print(
                f"  detecting corners: {pair_index}/{len(paired_names)}",
                flush=True,
            )

        left_image = cv2.imread(str(left_files[name]), cv2.IMREAD_GRAYSCALE)
        right_image = cv2.imread(str(right_files[name]), cv2.IMREAD_GRAYSCALE)

        if left_image is None or right_image is None:
            rejected_names.append(name)
            continue
        if left_image.shape != right_image.shape:
            rejected_names.append(name)
            continue

        current_size = (left_image.shape[1], left_image.shape[0])
        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            rejected_names.append(name)
            continue

        left_corners = find_corners(left_image)
        right_corners = find_corners(right_image)
        if left_corners is None or right_corners is None:
            rejected_names.append(name)
            continue

        # An unmarked checkerboard can be numbered in opposite directions in
        # the two images. Choose the ordering with the closer corresponding
        # corners before passing the pair to stereoCalibrate.
        same_order_distance = np.mean(
            np.linalg.norm(left_corners - right_corners, axis=2)
        )
        reversed_order_distance = np.mean(
            np.linalg.norm(left_corners - right_corners[::-1], axis=2)
        )
        if reversed_order_distance < same_order_distance:
            right_corners = right_corners[::-1].copy()
            reversed_right_names.append(name)

        object_points.append(object_template.copy())
        left_points.append(left_corners.astype(np.float32))
        right_points.append(right_corners.astype(np.float32))
        accepted_names.append(name)

    if len(object_points) < 3:
        raise RuntimeError(
            f"Only {len(object_points)} usable stereo pairs were found; need at least 3"
        )

    print(
        f"Calibrating with {len(object_points)} usable stereo pairs...",
        flush=True,
    )

    left_rms, left_camera_matrix, left_distortion, _, _ = cv2.calibrateCamera(
        object_points, left_points, image_size, None, None
    )
    right_rms, right_camera_matrix, right_distortion, _, _ = cv2.calibrateCamera(
        object_points, right_points, image_size, None, None
    )

    criteria = (
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
        100,
        1e-6,
    )
    stereo_rms, left_camera_matrix, left_distortion, right_camera_matrix, \
        right_distortion, rotation, translation, essential, fundamental = \
        cv2.stereoCalibrate(
            object_points,
            left_points,
            right_points,
            left_camera_matrix,
            left_distortion,
            right_camera_matrix,
            right_distortion,
            image_size,
            criteria=criteria,
            flags=cv2.CALIB_USE_INTRINSIC_GUESS,
        )

    left_rectification, right_rectification, left_projection, \
        right_projection, disparity_to_depth, left_roi, right_roi = \
        cv2.stereoRectify(
            left_camera_matrix,
            left_distortion,
            right_camera_matrix,
            right_distortion,
            image_size,
            rotation,
            translation,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=0,
        )

    lines = [
        "OpenCV stereo calibration\n",
        f"image_directory = {IMAGE_DIRECTORY}\n",
        f"image_size = {image_size[0]} x {image_size[1]}\n",
        f"board_inner_corners = {BOARD_COLUMNS} x {BOARD_ROWS}\n",
        f"square_size = {SQUARE_SIZE}\n",
        f"matching_pairs = {len(paired_names)}\n",
        f"accepted_pairs = {len(accepted_names)}\n",
        f"rejected_pairs = {len(rejected_names)}\n",
        f"right_corner_order_reversed = {len(reversed_right_names)}\n",
        f"left_monocular_rms = {left_rms:.12g}\n",
        f"right_monocular_rms = {right_rms:.12g}\n",
        f"stereo_rms = {stereo_rms:.12g}\n",
        f"baseline = {np.linalg.norm(translation):.12g}\n\n",
        matrix_text("left_camera_matrix", left_camera_matrix),
        matrix_text("left_distortion", left_distortion),
        matrix_text("right_camera_matrix", right_camera_matrix),
        matrix_text("right_distortion", right_distortion),
        matrix_text("rotation_left_to_right", rotation),
        matrix_text("translation_left_to_right", translation),
        matrix_text("essential_matrix", essential),
        matrix_text("fundamental_matrix", fundamental),
        matrix_text("left_rectification", left_rectification),
        matrix_text("right_rectification", right_rectification),
        matrix_text("left_projection", left_projection),
        matrix_text("right_projection", right_projection),
        matrix_text("disparity_to_depth_Q", disparity_to_depth),
        f"left_valid_roi = {left_roi}\n",
        f"right_valid_roi = {right_roi}\n\n",
        "Accepted image pairs:\n",
        *(f"  {name}\n" for name in accepted_names),
        "\nRejected image pairs:\n",
        *(f"  {name}\n" for name in rejected_names),
    ]

    OUTPUT_FILE.write_text("".join(lines), encoding="utf-8")

    print(f"Used {len(accepted_names)} of {len(paired_names)} matching pairs")
    print(f"Stereo RMS error: {stereo_rms:.6f}")
    print(f"Baseline: {np.linalg.norm(translation):.6f}")
    print(f"Wrote {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
