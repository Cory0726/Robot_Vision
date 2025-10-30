import cv2
import numpy as np
from pathlib import Path

# --- import your existing helpers (you already uploaded these) ---
# RGB intrinsics/undistortion
from undistortion_rgb_img import halcon_to_opencv_intrinsics  # citation below
# ToF intrinsics/undistortion helpers
from undistortion_tof_img import halcon_to_opencv_intrinsics_tof, build_undistort_maps  # citation below
# Pose parsing & T_RGB_from_ToF
from convert_tof_point_cloud_to_rgb import T_point2rgbframe  # citation below


# ---------------------------
# 1) Undistort helpers (no saving, pure array I/O)
# ---------------------------
def undistort_rgb_array(img_bgr):
    """Undistort an RGB/BGR image (expects HxWx3). Keeps same size using alpha=1."""
    h, w = img_bgr.shape[:2]
    K, dist = halcon_to_opencv_intrinsics()
    newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=1)
    map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, newK, (w, h), cv2.CV_32FC1)
    return cv2.remap(img_bgr, map1, map2, interpolation=cv2.INTER_LINEAR), newK

def undistort_tof_depth_array(depth_m):
    """
    Undistort ToF raw depth in *meters* (float32/float64), keep same size 640x480.
    Uses NEAREST to avoid depth averaging.
    """
    h, w = depth_m.shape[:2]
    Kt, distt = halcon_to_opencv_intrinsics_tof()
    map1, map2, newKt, _ = build_undistort_maps(Kt, distt, (w, h), alpha=1.0)
    out = cv2.remap(depth_m, map1, map2,
                    interpolation=cv2.INTER_NEAREST,
                    borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    return out, newKt


# ---------------------------
# 2) Project ToF depth to RGB frame
# ---------------------------
def project_tof_depth_to_rgb(depth_m_undist,
                             K_tof, K_rgb,
                             T_rgb_from_tof,
                             out_size=(1280, 1024)):
    """
    depth_m_undist: (H_tof, W_tof) depth in meters (after undistortion)
    K_tof, K_rgb:   3x3 intrinsics (after undistortion if you used newK)
    T_rgb_from_tof: 4x4 transform (ToF->RGB)
    out_size:       (W, H) of RGB image

    Returns:
      depth_rgb_m: (H_rgb, W_rgb) float32 depth map in meters, aligned to RGB
    """
    Ht, Wt = depth_m_undist.shape
    Wrgb, Hrgb = out_size

    # build pixel grid in ToF image
    u, v = np.meshgrid(np.arange(Wt), np.arange(Ht))
    z = depth_m_undist.astype(np.float32)
    valid = z > 0
    if not np.any(valid):
        return np.zeros((Hrgb, Wrgb), dtype=np.float32)

    u = u[valid].astype(np.float32)
    v = v[valid].astype(np.float32)
    z = z[valid]

    fx_t, fy_t = K_tof[0, 0], K_tof[1, 1]
    cx_t, cy_t = K_tof[0, 2], K_tof[1, 2]

    # back-project ToF pixels -> 3D (ToF camera frame)
    x_t = (u - cx_t) / fx_t * z
    y_t = (v - cy_t) / fy_t * z
    pts_tof = np.stack([x_t, y_t, z], axis=1)  # Nx3

    # transform to RGB camera frame
    R = T_rgb_from_tof[:3, :3]
    t = T_rgb_from_tof[:3, 3]
    pts_rgb = pts_tof @ R.T + t[None, :]

    X, Y, Z = pts_rgb[:, 0], pts_rgb[:, 1], pts_rgb[:, 2]
    # remove points behind the RGB camera or too close
    ok = Z > 1e-6
    if not np.any(ok):
        return np.zeros((Hrgb, Wrgb), dtype=np.float32)
    X, Y, Z = X[ok], Y[ok], Z[ok]

    fx_r, fy_r = K_rgb[0, 0], K_rgb[1, 1]
    cx_r, cy_r = K_rgb[0, 2], K_rgb[1, 2]
    u_r = (X / Z) * fx_r + cx_r
    v_r = (Y / Z) * fy_r + cy_r

    # discretize to pixel indices
    u_i = np.round(u_r).astype(np.int32)
    v_i = np.round(v_r).astype(np.int32)

    inside = (u_i >= 0) & (u_i < Wrgb) & (v_i >= 0) & (v_i < Hrgb)
    u_i, v_i, Z = u_i[inside], v_i[inside], Z[inside]

    # z-buffer splat: keep nearest Z per RGB pixel
    depth_rgb = np.zeros((Hrgb, Wrgb), dtype=np.float32)
    far = np.full((Hrgb, Wrgb), np.inf, dtype=np.float32)
    lin = v_i * Wrgb + u_i
    # for same pixel, keep min Z
    np.minimum.at(far, (v_i, u_i), Z)
    # fill where we saw anything
    hit = np.isfinite(far)
    depth_rgb[hit] = far[hit]
    return depth_rgb


# ---------------------------
# 3) Fuse into RGBD-4ch tensor (BGR + depth_m)
# ---------------------------
def fuse_rgb_and_tof_depth(rgb_bgr, depth_tof_m):
    """
    End-to-end:
      - undistort RGB & ToF depth
      - use extrinsics to align ToF depth onto RGB
      - pack into 4-channel (H=1024, W=1280): B,G,R,Depth(m)
    """
    # 1) undistort RGB
    rgb_undist, K_rgb = undistort_rgb_array(rgb_bgr)             # size 1280x1024 (keep)
    # 2) undistort ToF depth (expects meters)
    depth_undist, K_tof = undistort_tof_depth_array(depth_tof_m) # size 640x480

    # 3) extrinsic: ToF -> RGB
    T_rgb_from_tof = T_point2rgbframe()

    # 4) project ToF depth onto RGB canvas
    Hrgb, Wrgb = rgb_undist.shape[:2]
    depth_on_rgb_m = project_tof_depth_to_rgb(depth_undist, K_tof, K_rgb, T_rgb_from_tof,
                                              out_size=(Wrgb, Hrgb))

    # 5) stack B,G,R,Depth(m). Empty places remain zero by construction.
    depth_on_rgb_m = depth_on_rgb_m.astype(np.float32)  # meters
    rgbd = np.dstack([rgb_undist.astype(np.uint8), depth_on_rgb_m])

    return rgbd, depth_on_rgb_m, rgb_undist


# ---------------------------
# 4) Example usage
# ---------------------------
if __name__ == "__main__":
    # Load your images (examples)
    rgb = cv2.imread("rgb_img.png", cv2.IMREAD_COLOR)           # 1280x1024x3
    depth = cv2.imread("depth_raw.png", cv2.IMREAD_UNCHANGED)  # float meters (recommended)
    print(depth.shape, depth.dtype)
    # If your depth is uint16 in millimeters, convert first:
    if depth.dtype == np.uint16:
        depth = depth.astype(np.float32) / 1000.0

    rgbd, depth_aligned_m, rgb_undist = fuse_rgb_and_tof_depth(rgb, depth)

    # Save: RGBD as two files (RGB PNG + depth PNG in mm for inspection)
    cv2.imwrite("rgb_undist.png", rgb_undist)
    depth_mm = np.clip(depth_aligned_m * 1000.0, 0, 65535).astype(np.uint16)
    print(depth_mm.shape, depth_mm.dtype, depth_mm.max(), depth_mm.min())
    cv2.imwrite("depth_on_rgb_mm.png", depth_mm)
    check = cv2.imread("depth_on_rgb_mm.png", cv2.IMREAD_UNCHANGED)
    print(check.shape, check.dtype, check.max(), check.min())
    gray_img = (check / check.max() * 255.0).astype(np.uint8)
    heatmap = cv2.applyColorMap(gray_img, cv2.COLORMAP_JET)
    cv2.imwrite("depth_on_rgb_mm_heatmap.png", heatmap)
    # (Optionally) save a 4-ch .npy tensor
    np.save("rgbd_4ch.npy", rgbd)  # shape: (1024,1280,4)  [B,G,R,Depth(m)]
