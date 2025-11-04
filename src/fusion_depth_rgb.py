import cv2
import numpy as np

import basler_rgb_cam_grab
import basler_tof_cam_grab
# ---------------------- Config ----------------------
CALIB_XML = r"./calibration/calibration_<blazeSerial>_<colorDeviceID>.xml"
DEPTH_PATH = r"./depth.png"   # or .tiff/.exr... your raw depth
COLOR_PATH = r"./rgb.png"     # BGR or RGB (we will assume BGR uint8)

# Set this if you know your units. If your raw depth is uint16 in millimeters (typical),
# set DEPTH_SCALE_M = 0.001. If it's already meters in float, set 1.0.
DEPTH_SCALE_M = None  # None = auto: uint16 -> 0.001, float -> 1.0

# Interpolation when sampling color image at projected coords: 'nearest' or 'bilinear'
INTERP = "nearest"
# ----------------------------------------------------


# ------------ Utilities (English comments) ----------
def load_calibration(xml_path):
    """
    Load intrinsic/extrinsic matrices from the calibration XML.

    Returns:
        Kc (3x3), dc (Nx1), Kd (3x3), dd (Nx1), R (3x3), T (3x1)
        where:
          Kc, dc: color intrinsics, distortion
          Kd, dd: blaze (depth) intrinsics, distortion (often zeros)
          R, T:   transform from depth(blaze) frame to color frame
    """
    fs = cv2.FileStorage(xml_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(f"Cannot open calibration file: {xml_path}")

    Kc = fs.getNode("colorCameraMatrix").mat()
    dc = fs.getNode("colorDistortion").mat()
    Kd = fs.getNode("blazeCameraMatrix").mat()  # may exist in your XML
    dd = fs.getNode("blazeDistortion").mat()    # may be zeros in your sample
    R  = fs.getNode("rotation").mat()
    T  = fs.getNode("translation").mat()
    fs.release()

    if Kd is None or Kd.size == 0:
        raise ValueError("Missing 'blazeCameraMatrix' in XML (depth intrinsics are required).")

    if dd is None or dd.size == 0:
        dd = np.zeros((1,5), dtype=np.float32)

    # Ensure proper shapes
    Kc = Kc.astype(np.float32)
    dc = dc.astype(np.float32).reshape(-1,1)
    Kd = Kd.astype(np.float32)
    dd = dd.astype(np.float32).reshape(-1,1)
    R  = R.astype(np.float32)
    T  = T.astype(np.float32).reshape(3,1)

    return Kc, dc, Kd, dd, R, T


def depth_to_meters(depth_raw, depth_scale_m=None):
    """
    Convert raw depth to meters as float32.
    If depth_scale_m is None, auto-detect:
        - uint16: assume millimeters -> 0.001
        - float:  assume already in meters -> 1.0
    """
    if depth_scale_m is None:
        if depth_raw.dtype == np.uint16:
            depth_scale_m = 0.001
        else:
            depth_scale_m = 1.0

    depth_m = depth_raw.astype(np.float32) * float(depth_scale_m)
    # Treat non-positive as invalid
    depth_m[~np.isfinite(depth_m)] = 0.0
    depth_m[depth_m <= 0] = 0.0
    return depth_m


# def backproject_depth_to_3d(depth_m, Kd):
#     """
#     Back-project a depth image (meters) to an organized point cloud in the depth camera frame.
#
#     Args:
#         depth_m: (Hd, Wd) float32, meters
#         Kd:      (3x3) intrinsics of the depth/blaze camera
#
#     Returns:
#         pts_3d: (Hd, Wd, 3) float32, XYZ in meters
#     """
#     Hd, Wd = depth_m.shape
#     fx, fy = Kd[0,0], Kd[1,1]
#     cx, cy = Kd[0,2], Kd[1,2]
#
#     # Create a pixel grid (u = x, v = y)
#     u = np.arange(Wd, dtype=np.float32)
#     v = np.arange(Hd, dtype=np.float32)
#     uu, vv = np.meshgrid(u, v)
#
#     Z = depth_m
#     X = (uu - cx) * Z / fx
#     Y = (vv - cy) * Z / fy
#
#     pts = np.dstack((X, Y, Z)).astype(np.float32)  # (Hd, Wd, 3)
#     return pts


def project_points_to_color(pts3d, R, T, Kc, dc, color_img, interp="nearest"):
    """
    Project organized 3D points (depth frame) into the color camera and sample color.

    Args:
        pts3d:     (Hd, Wd, 3) float32, XYZ in depth frame
        R, T:      color <- depth transform (3x3, 3x1)
        Kc, dc:    color intrinsics and distortion
        color_img: (Hc, Wc, 3) uint8 BGR
        interp:    'nearest' or 'bilinear'

    Returns:
        color_on_depth: (Hd, Wd, 3) uint8, BGR on depth grid (zeros where invalid)
        valid_mask:     (Hd, Wd) bool, True if sampled inside color bounds and Z>0
    """
    Hd, Wd, _ = pts3d.shape
    Hc, Wc = color_img.shape[:2]

    pts = pts3d.reshape(-1, 3)

    # OpenCV can take a 3x3 rotation matrix in place of rvec; cv2 will convert internally
    img_pts, _ = cv2.projectPoints(pts, R, T, Kc, dc)  # -> (N,1,2)
    img_pts = img_pts.reshape(-1, 2)

    if interp == "nearest":
        u = np.rint(img_pts[:,0]).astype(np.int32)  # x (col)
        v = np.rint(img_pts[:,1]).astype(np.int32)  # y (row)
        Z = pts[:,2]
        valid = (Z > 0) & (u >= 0) & (u < Wc) & (v >= 0) & (v < Hc)

        out = np.zeros((Hd*Wd, 3), dtype=np.uint8)
        out[valid] = color_img[v[valid], u[valid]]
        out = out.reshape(Hd, Wd, 3)
        return out, valid.reshape(Hd, Wd)

    elif interp == "bilinear":
        # Bilinear sampling on float coords
        u = img_pts[:,0]
        v = img_pts[:,1]
        Z = pts[:,2]
        # bounds for sampling window
        u0 = np.floor(u).astype(np.int32)
        v0 = np.floor(v).astype(np.int32)
        u1 = u0 + 1
        v1 = v0 + 1

        valid = (Z > 0) & (u0 >= 0) & (v0 >= 0) & (u1 < Wc) & (v1 < Hc)
        out = np.zeros((Hd*Wd, 3), dtype=np.float32)

        # weights
        du = (u - u0).astype(np.float32)
        dv = (v - v0).astype(np.float32)
        w00 = (1-du)*(1-dv)
        w10 = du*(1-dv)
        w01 = (1-du)*dv
        w11 = du*dv

        idx = np.where(valid)[0]
        uu0, vv0 = u0[idx], v0[idx]
        uu1, vv1 = u1[idx], v1[idx]

        c00 = color_img[vv0, uu0].astype(np.float32)
        c10 = color_img[vv0, uu1].astype(np.float32)
        c01 = color_img[vv1, uu0].astype(np.float32)
        c11 = color_img[vv1, uu1].astype(np.float32)

        out[idx] = (c00*w00[idx,None] + c10*w10[idx,None] +
                    c01*w01[idx,None] + c11*w11[idx,None])
        out = np.clip(out, 0, 255).astype(np.uint8).reshape(Hd, Wd, 3)
        return out, valid.reshape(Hd, Wd)

    else:
        raise ValueError("interp must be 'nearest' or 'bilinear'")


def depth_m_to_uint16_mm(depth_m):
    """Meters -> uint16 millimeters; invalid(<=0 or nan) -> 0."""
    d = depth_m.copy()
    invalid = (~np.isfinite(d)) | (d <= 0)
    d[invalid] = 0.0
    d_mm = np.clip(np.round(d * 1000.0), 0, 65535).astype(np.uint16)
    return d_mm


# -------------------- Main pipeline -----------------
def main():
    # 1) Load calibration
    Kc, dc, Kd, dd, R, T = load_calibration(CALIB_XML)

    # 2) Read inputs
    # color = cv2.imread(COLOR_PATH, cv2.IMREAD_COLOR)       # (Hc,Wc,3) BGR uint8
    # if color is None:
    #     raise FileNotFoundError(COLOR_PATH)
    color = basler_rgb_cam_grab.grab_one_rgb_img()

    # raw depth: use IMREAD_UNCHANGED to preserve bit depth
    depth_raw = cv2.imread(DEPTH_PATH, cv2.IMREAD_UNCHANGED)
    if depth_raw is None:
        raise FileNotFoundError(DEPTH_PATH)

    # # 3) Convert depth to meters (float32)
    # depth_m = depth_to_meters(depth_raw, depth_scale_m=DEPTH_SCALE_M)  # (Hd,Wd)
    #
    # # 4) Backproject to 3D in depth frame (organized point cloud)
    # pts3d = backproject_depth_to_3d(depth_m, Kd)  # (Hd,Wd,3)
    pts3d = basler_tof_cam_grab.grab_one_point_cloud()
    # 5) Project to color and sample the RGB on the depth grid
    aligned_rgb, valid_mask = project_points_to_color(
        pts3d, R, T, Kc, dc, color, interp=INTERP
    )  # (Hd,Wd,3), (Hd,Wd)

    # # 6) Produce aligned uint16 depth in millimeters
    # aligned_depth_mm = depth_m_to_uint16_mm(depth_m)  # (Hd,Wd) uint16
    aligned_depth_mm = basler_tof_cam_grab.pcl_to_rawdepth(pts3d)
    # 7) Optional: visualization
    depth_vis = cv2.applyColorMap(
        cv2.convertScaleAbs(aligned_depth_mm, alpha=1.0/16.0),  # quick stretch for viewing
        cv2.COLORMAP_TURBO
    )
    overlay = cv2.addWeighted(aligned_rgb, 0.6, depth_vis, 0.4, 0)

    cv2.imshow("Aligned RGB (depth grid)", aligned_rgb)
    cv2.imshow("Aligned Depth colormap (depth grid)", depth_vis)
    cv2.imshow("Overlay", overlay)
    cv2.waitKey(0)

    # 8) Save outputs if you want
    cv2.imwrite("aligned_rgb.png", aligned_rgb)
    cv2.imwrite("aligned_depth_mm.png", aligned_depth_mm)   # 16-bit PNG
    cv2.imwrite("overlay.png", overlay)

    print("Done. aligned_rgb.png / aligned_depth_mm.png saved.")

if __name__ == "__main__":
    main()
