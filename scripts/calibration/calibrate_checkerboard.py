"""
Calibration for a (partially visible) Duckietown checkerboard.
We mask part with white paper and use a smaller inner-corner grid.
Requires one example image in the same folder.
"""

import cv2 as cv
import numpy as np
import os
from datetime import datetime, timezone
import yaml
# ----------------- CONFIG -----------------
# Get the directory where the script is located
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Look for image.png in the same directory
IMAGE_PATH = os.path.join(SCRIPT_DIR, "image.png")
CHECKERBOARD_SIZE = (5, 3)      # of *inner* corners (visible patch)
SQUARE_SIZE_MM = 31               # square size (mm)
SQUARE_SIZE_M  = SQUARE_SIZE_MM / 1000.0

# Camera intrinsics (fill these!)
# --- Camera intrinsics from YAML ---
K = np.array([
    [321.926466, 0.000000, 164.226837],
    [0.000000, 323.331045, 116.629331],
    [0.000000, 0.000000, 1.000000]
], dtype=float)

dist = np.array([0.096658, -0.282953, -0.003192, -0.002148, 0.000000], dtype=float)


# --- Checkerboard corners in BASE frame (mm). Origin = left_bot inner corner (0,0)
# base x : forward, base y : left, base z : up (example convention)
right_bot = np.array([160, -62, 0.0])  # x, y, z (mm)
right_top = np.array([346, -62, 0.0])
left_bot  = np.array([160,  62, 0.0])
left_top  = np.array([346,  62, 0.0])
# ------------------------------------------
def R_to_quat(R):
    # returns (w, x, y, z)
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2,1] - R[1,2]) / s
        y = (R[0,2] - R[2,0]) / s
        z = (R[1,0] - R[0,1]) / s
    else:
        i = np.argmax([R[0,0], R[1,1], R[2,2]])
        if i == 0:
            s = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            w = (R[2,1] - R[1,2]) / s
            x = 0.25 * s
            y = (R[0,1] + R[1,0]) / s
            z = (R[0,2] + R[2,0]) / s
        elif i == 1:
            s = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            w = (R[0,2] - R[2,0]) / s
            x = (R[0,1] + R[1,0]) / s
            y = 0.25 * s
            z = (R[1,2] + R[2,1]) / s
        else:
            s = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            w = (R[1,0] - R[0,1]) / s
            x = (R[0,2] + R[2,0]) / s
            y = (R[1,2] + R[2,1]) / s
            z = 0.25 * s
    return np.array([w, x, y, z], dtype=float)

def write_transforms_yaml(path, T_base_cam, T_cam_base):
    
    R_bc = T_base_cam[:3,:3]; t_bc = T_base_cam[:3,3]
    R_cb = T_cam_base[:3,:3]; t_cb = T_cam_base[:3,3]
    q_bc = R_to_quat(R_bc)
    q_cb = R_to_quat(R_cb)

    # Create dictionary structure for YAML
    data = {
        "stamp": datetime.now(timezone.utc).isoformat(),
        "frames": {
            "parent": "base",
            "child": "camera"
        },
        "T_base_cam": {
            "matrix": T_base_cam.tolist()
        },
        "T_cam_base": {
            "matrix": T_cam_base.tolist()
        },
        "rotation_base_cam": {
            "w": float(q_bc[0]),
            "x": float(q_bc[1]),
            "y": float(q_bc[2]),
            "z": float(q_bc[3])
        },
        "translation_base_cam": {
            "x": float(t_bc[0]),
            "y": float(t_bc[1]),
            "z": float(t_bc[2])
        },
        "rotation_cam_base": {
            "w": float(q_cb[0]),
            "x": float(q_cb[1]),
            "y": float(q_cb[2]),
            "z": float(q_cb[3])
        },
        "translation_cam_base": {
            "x": float(t_cb[0]),
            "y": float(t_cb[1]),
            "z": float(t_cb[2])
        }
    }

    # Write to YAML file
    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)
    
    print(f"Wrote transforms -> {path}")

def make_object_points(cols, rows, square):
    objp = np.zeros((rows*cols, 3), np.float32)
    # grid in (x=cols, y=rows) with origin at left_bot corner
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square
    return objp

def solve_pnp(image_path, checker_size, square_m, K, dist):
    cols, rows = checker_size
    img = cv.imread(image_path)
    assert img is not None, f"Cannot read {image_path}"
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    flags = (cv.CALIB_CB_ADAPTIVE_THRESH |
             cv.CALIB_CB_NORMALIZE_IMAGE |
             cv.CALIB_CB_FAST_CHECK)

    ok, corners = cv.findChessboardCorners(gray, (cols, rows), flags)
    assert ok, "Checkerboard not found. Check size (cols,rows) and visibility."
    corners = cv.cornerSubPix(
        gray, corners, (11,11), (-1,-1),
        (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 50, 1e-3)
    )

    objp = make_object_points(cols, rows, square_m)
    ok, rvec, tvec = cv.solvePnP(objp, corners, K, dist, flags=cv.SOLVEPNP_ITERATIVE)
    assert ok, "solvePnP failed"
    R, _ = cv.Rodrigues(rvec)
    t = tvec.reshape(3)
    T_cam_board = np.eye(4); T_cam_board[:3,:3] = R; T_cam_board[:3,3] = t
    return img, corners, T_cam_board

def base_from_four_points(left_bot_mm, right_bot_mm, left_top_mm, right_top_mm):
    # define board frame in BASE: origin at left_bot (0,0,0), X→right, Y→up along board
    lb = left_bot_mm / 1000.0
    rb = right_bot_mm / 1000.0
    lt = left_top_mm  / 1000.0
    # Axes
    x_dir = rb - lb
    y_dir = lt - lb
    x_axis = x_dir / np.linalg.norm(x_dir)
    y_axis = y_dir / np.linalg.norm(y_dir)
    z_axis = np.cross(x_axis, y_axis)
    z_axis /= np.linalg.norm(z_axis)
    # Re-orthogonalize y to ensure right-handed frame
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)

    T_base_board = np.eye(4)
    T_base_board[:3, :3] = np.column_stack([x_axis, y_axis, z_axis])
    T_base_board[:3,  3] = lb
    return T_base_board

def invert(T):
    R = T[:3,:3]; t = T[:3,3]
    Ti = np.eye(4)
    Ti[:3,:3] = R.T
    Ti[:3,3] = -R.T @ t
    return Ti

def main():
    img, corners, T_cam_board = solve_pnp(IMAGE_PATH, CHECKERBOARD_SIZE, SQUARE_SIZE_M, K, dist)
    T_board_cam = invert(T_cam_board)

    T_base_board = base_from_four_points(left_bot, right_bot, left_top, right_top)
    T_base_cam = T_base_board @ T_board_cam
    T_cam_base = invert(T_base_cam)

    np.set_printoptions(precision=6, suppress=True)
    print("T_cam_board =\n", T_cam_board)
    print("T_base_board =\n", T_base_board)
    print("T_base_cam =\n", T_base_cam)
    print("T_cam_base =\n", T_cam_base)

    # Optional visualization
    vis = cv.drawChessboardCorners(img.copy(), CHECKERBOARD_SIZE, corners, True)
    # Save visualization in the same folder as the script
    output_path = os.path.join(SCRIPT_DIR, "checker_detect_vis.jpg")
    cv.imwrite(output_path, vis)
    print("Saved detection viz -> checker_detect_vis.jpg")

    # Define output path in config/ folder relative to working directory
    config_dir = os.path.join(os.getcwd(), "config")
    # Create config directory if it doesn't exist
    os.makedirs(config_dir, exist_ok=True)
    OutputYamlPath = os.path.join(config_dir, "transforms.yaml")
    # Write the transforms to the YAML file
    write_transforms_yaml(OutputYamlPath, T_base_cam, T_cam_base)

if __name__ == "__main__":
    main()
