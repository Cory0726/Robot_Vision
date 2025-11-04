import numpy as np
from pathlib import Path

# ---------- Pose parsing utilities ----------

def _euler_zyx_deg_to_R(rx_deg, ry_deg, rz_deg):
    """HALCON f 2 uses Euler angles in degrees with Z-Y-X order: R = Rz * Ry * Rx."""
    rx, ry, rz = np.deg2rad([rx_deg, ry_deg, rz_deg])
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx),  np.cos(rx)]])
    Ry = np.array([[ np.cos(ry), 0, np.sin(ry)],
                   [0,           1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz),  np.cos(rz), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx

def _rodrigues_to_R(rx, ry, rz):
    """HALCON f 1 uses Rodrigues (radians). Convert axis-angle vector to rotation matrix."""
    r = np.array([rx, ry, rz], dtype=float)
    theta = np.linalg.norm(r)
    if theta < 1e-12:
        return np.eye(3)
    k = r / theta
    K = np.array([[0, -k[2], k[1]],
                  [k[2], 0, -k[0]],
                  [-k[1], k[0], 0]])
    R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)
    return R

def read_halcon_pose_dat(path):
    """
    Read a HALCON pose .dat/.pose file.
    Returns a 4x4 homogeneous transform T_dest_from_src (i.e., frame on header: 'flange in CAMERA' -> T_CAMERA_from_FLANGE).
    Supported formats:
      - 'f 2'  -> Euler angles [deg], ZYX order
      - 'f 1'  -> Rodrigues vector [rad]
    """
    text = Path(path).read_text(encoding='utf-8', errors='ignore').replace(',', ' ')
    # Find f type
    f_type = None
    for line in text.splitlines():
        ls = line.strip().lower()
        if ls.startswith('f'):
            parts = ls.split()
            if len(parts) >= 2 and parts[0] == 'f':
                f_type = int(float(parts[1]))
                break
    # Parse rotation 'r ...'
    r_vals, t_vals = None, None
    for line in text.splitlines():
        ls = line.strip().replace(',', ' ')
        if ls.startswith('r ') or ls.lower().startswith('r '):
            r_vals = [float(x) for x in ls.split()[1:4]]
        if ls.startswith('t ') or ls.lower().startswith('t '):
            t_vals = [float(x) for x in ls.split()[1:4]]
    if f_type is None or r_vals is None or t_vals is None:
        raise ValueError(f"Cannot parse pose file: {path}")
    if f_type == 2:
        R = _euler_zyx_deg_to_R(*r_vals)
    elif f_type == 1:
        R = _rodrigues_to_R(*r_vals)
    else:
        raise NotImplementedError(f"Unsupported HALCON pose representation f={f_type}")

    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3]  = np.array(t_vals, dtype=float)  # meters
    return T

# ---------- Compose Cam-to-Cam transform ----------

def compose_to_rgb_cam(T_camRGB_from_flange, T_camToF_from_flange):
    """
    Given 'flange in RGB cam'  -> T_RGB_from_FLANGE
          'flange in ToF cam'  -> T_ToF_from_FLANGE
    We want T_RGB_from_ToF.
    T_RGB_from_ToF = T_RGB_from_FLANGE * inv(T_ToF_from_FLANGE)
    """
    T_rgb_f = T_camRGB_from_flange
    T_tof_f = T_camToF_from_flange
    T_rgb_from_tof = T_rgb_f @ np.linalg.inv(T_tof_f)
    return T_rgb_from_tof

# ---------- Apply transform to a point cloud ----------

def transform_points(T, pts):
    """
    Apply 4x4 transform to Nx3 points.
    """
    pts = np.asarray(pts, dtype=float)
    ones = np.ones((pts.shape[0], 1), dtype=float)
    P = np.hstack([pts, ones])
    Q = (T @ P.T).T
    return Q[:, :3]

def T_point2rgbframe():
    # Load the two poses you uploaded (edit the paths if needed)
    pose_rgb_path = "./calibration_result/flange_in_RGB_cam_SN24747625.dat"
    pose_tof_path = "./calibration_result/flange_in_ToF_cam_SN24945819.dat"

    T_RGB_from_FLANGE = read_halcon_pose_dat(pose_rgb_path)
    T_ToF_from_FLANGE = read_halcon_pose_dat(pose_tof_path)

    # Compute Cam-ToF -> Cam-RGB transform
    T_RGB_from_ToF = compose_to_rgb_cam(T_RGB_from_FLANGE, T_ToF_from_FLANGE)
    return T_RGB_from_ToF



