# -*- coding: utf-8 -*-
"""Perception: item dimensions in millimeters from the top-down depth frame (day 2, P3).

The core is a pure numpy function measure_dims_mm(); the depth-PNG loading and the
cv2 overlay live around it so the geometry is testable without ROS or OpenCV.

Scene calibration mirrors sim/worlds/cell.sdf (that world file is the single source):
camera looks straight down from z=1.9 m, belt top at z=0.4 m, RGBD 640x480,
horizontal_fov 1.05 rad. At runtime the node should prefer /camera/camera_info;
these constants are the offline calibration for the saved frame. If the world
changes, update here.
"""
import numpy as np

CAMERA_Z_M = 1.9      # cell.sdf: camera model pose z
BELT_TOP_Z_M = 0.4    # cell.sdf: belt top surface (platform top)
IMG_W, IMG_H = 640, 480
HFOV_RAD = 1.05       # cell.sdf: camera horizontal_fov

# depth of the empty belt surface seen from the camera
BELT_DEPTH_M = CAMERA_Z_M - BELT_TOP_Z_M
# square pixels: fy == fx (verified against the real frame, ±3 mm on Короб 300)
FX = FY = (IMG_W / 2.0) / np.tan(HFOV_RAD / 2.0)

_MIN_ITEM_PX = 200    # ignore specks; a real item covers thousands of pixels


def _item_mask(depth_m, belt_depth_m, margin_m):
    """Boolean mask of pixels standing above the belt by more than margin."""
    return (depth_m > 0) & (depth_m < belt_depth_m - margin_m)


def measure_dims_mm(depth_m, belt_depth_m=BELT_DEPTH_M, fx=FX, fy=FY, margin_m=0.02):
    """Three dimensions (mm, sorted descending) of the single item on the belt.

    depth_m: HxW depth image in meters (0 = no return). Returns None if no item.
    Two lateral dims come from the item's pixel bounding box at its top-face
    depth; the third (height) from how far the top rises above the belt.
    """
    mask = _item_mask(depth_m, belt_depth_m, margin_m)
    ys, xs = np.where(mask)
    if xs.size < _MIN_ITEM_PX:
        return None
    top_depth_m = float(np.median(depth_m[mask]))
    w_px = int(xs.max() - xs.min() + 1)
    h_px = int(ys.max() - ys.min() + 1)
    dx_mm = w_px * top_depth_m / fx * 1000.0
    dy_mm = h_px * top_depth_m / fy * 1000.0
    dz_mm = (belt_depth_m - top_depth_m) * 1000.0
    return sorted([dx_mm, dy_mm, dz_mm], reverse=True)


def item_bbox_px(depth_m, belt_depth_m=BELT_DEPTH_M, margin_m=0.02):
    """(x0, y0, x1, y1) pixel bounding box of the item, or None."""
    mask = _item_mask(depth_m, belt_depth_m, margin_m)
    ys, xs = np.where(mask)
    if xs.size < _MIN_ITEM_PX:
        return None
    return int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())


def load_depth_png(path):
    """Load a uint16-millimeter depth PNG (as dumped by scripts/dump_camera.py) to meters."""
    import cv2

    mm = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if mm is None:
        raise FileNotFoundError(path)
    return mm.astype(np.float64) / 1000.0


def save_overlay(depth_m, out_path, belt_depth_m=BELT_DEPTH_M):
    """Draw the measured bbox and dimensions over the depth frame (Karpathy #4)."""
    import cv2

    dims = measure_dims_mm(depth_m, belt_depth_m)
    bbox = item_bbox_px(depth_m, belt_depth_m)
    gray = np.clip((depth_m - 1.2) / (2.0 - 1.2), 0, 1)  # belt/floor band -> readable
    vis = cv2.cvtColor((gray * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    if bbox is not None:
        x0, y0, x1, y1 = bbox
        cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 2)
        label = " x ".join(f"{d:.0f}" for d in dims) + " mm"
        cv2.putText(vis, label, (x0, max(y0 - 8, 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.imwrite(str(out_path), vis)
    return dims
