import cv2
import numpy as np
from pathlib import Path

# ---------- 1) Convert HALCON intrinsics -> OpenCV intrinsics for the ToF camera ----------
def halcon_to_opencv_intrinsics_tof(
    # Pixel size [micrometers]
    Sx_um=8.29572, Sy_um=8.3,
    # Focal length [mm]
    f_mm=4.32481,
    # Principal point [pixels]
    Cx_px=329.512, Cy_px=225.884,
    # HALCON distortion coefficients (units shown in your screenshot)
    # Radial: [1/m^2, 1/m^4, 1/m^6]
    K1=-40.7214, K2=-1.66096e8, K3=2.11721e13,
    # Tangential 2nd order (HALCON labels 1/m^2)
    P1=0.32381, P2=0.538484
):
    """
    Build OpenCV cameraMatrix (in pixels) and distCoeffs from HALCON parameters.
    NOTE:
      - HALCON radial units: K1 [1/m^2], K2 [1/m^4], K3 [1/m^6]
      - HALCON tangential units here: P1, P2 [1/m^2]  <-- from your GUI label
      - OpenCV expects *dimensionless* coefficients in normalized coordinates
        so we convert by multiplying with powers of focal length (in meters):
          k1 = K1 * f^2,  k2 = K2 * f^4,  k3 = K3 * f^6
          p1 = P1 * f^2,  p2 = P2 * f^2
    """

    # --- Units to meters ---
    f_m  = f_mm / 1000.0          # mm -> m
    Sx_m = Sx_um * 1e-6           # um -> m
    Sy_m = Sy_um * 1e-6

    # --- Focal length in pixels ---
    fx = f_m / Sx_m
    fy = f_m / Sy_m
    cx, cy = Cx_px, Cy_px

    cameraMatrix = np.array([[fx, 0,  cx],
                             [0,  fy, cy],
                             [0,  0,   1]], dtype=np.float64)

    # --- HALCON -> OpenCV coefficient conversion (dimensionless) ---
    k1 = K1 * (f_m ** 2)
    k2 = K2 * (f_m ** 4)
    k3 = K3 * (f_m ** 6)
    p1 = P1 * (f_m ** 2)   # IMPORTANT: P1,P2 shown as 1/m^2 in your UI
    p2 = P2 * (f_m ** 2)

    distCoeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float64)
    return cameraMatrix, distCoeffs


# ---------- 2) Undistortion helpers ----------
def build_undistort_maps(K, dist, size, alpha=1.0):
    """
    Create undistortion/rectification maps once, then reuse for remapping.
    - size: (width, height) in pixels
    - alpha:
        0.0 -> crop all black borders (smaller FOV, no black edges)
        1.0 -> keep full FOV (same resolution, black edges may appear)
        (0~1 for trade-off)
    Returns: map1, map2, newK, roi
    """
    w, h = size
    newK, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha)
    map1, map2 = cv2.initUndistortRectifyMap(
        K, dist, R=None, newCameraMatrix=newK, size=(w, h), m1type=cv2.CV_32FC1
    )
    return map1, map2, newK, roi


def undistort_tof_image(
    img_path,
    out_path=None,
    alpha=1.0,
    crop=False,
    interpolation="linear"
):
    """
    Undistort a ToF intensity image (or any 2D image).
    - alpha: see build_undistort_maps()
    - crop:  if True, crop to valid ROI; if False, keep original size (may have black edges)
    - interpolation: "linear" for grayscale/intensity, "nearest" recommended for label-like maps
    """
    # 1) Load image
    img = cv2.imread(str(img_path), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise FileNotFoundError(f"Cannot read image: {img_path}")
    h, w = img.shape[:2]

    # 2) Get intrinsics/distortion for ToF and undistort maps
    K, dist = halcon_to_opencv_intrinsics_tof()
    map1, map2, newK, roi = build_undistort_maps(K, dist, (w, h), alpha=alpha)

    # 3) Choose interpolation
    interp = cv2.INTER_LINEAR if interpolation.lower().startswith("lin") else cv2.INTER_NEAREST

    # 4) Remap
    undist = cv2.remap(img, map1, map2, interpolation=interp)

    # 5) Optional crop to valid region
    if crop:
        x, y, rw, rh = roi
        if rw > 0 and rh > 0:
            undist = undist[y:y+rh, x:x+rw]

    # 6) Save
    if out_path is None:
        out_path = Path(img_path).with_name(Path(img_path).stem + "_undistorted.png")
    cv2.imwrite(str(out_path), undist)

    return {"cameraMatrix": K, "distCoeffs": dist, "newCameraMatrix": newK,
            "roi": roi, "out_path": str(out_path)}


def undistort_tof_depth(
    depth_path,
    out_path=None,
    alpha=1.0,
    crop=False,
    invalid_value=0
):
    """
    Undistort a ToF depth map (e.g., in millimeters).
    IMPORTANT:
      - Use NEAREST interpolation to avoid averaging depth values.
      - invalid_value: pixels remapped from outside source will be set to this.
    """
    depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
    depth = depth / 1000.0
    if depth is None:
        raise FileNotFoundError(f"Cannot read depth: {depth_path}")
    h, w = depth.shape[:2]

    K, dist = halcon_to_opencv_intrinsics_tof()
    map1, map2, newK, roi = build_undistort_maps(K, dist, (w, h), alpha=alpha)

    undist = cv2.remap(depth, map1, map2, interpolation=cv2.INTER_NEAREST,
                       borderMode=cv2.BORDER_CONSTANT, borderValue=invalid_value)

    if crop:
        x, y, rw, rh = roi
        if rw > 0 and rh > 0:
            undist = undist[y:y+rh, x:x+rw]
    undist = np.clip(undist * 1000.0,0,65535).astype(np.uint16)
    if out_path is None:
        out_path = Path(depth_path).with_name(Path(depth_path).stem + "_undistorted.png")
    cv2.imwrite(str(out_path), undist)

    return {"cameraMatrix": K, "distCoeffs": dist, "newCameraMatrix": newK,
            "roi": roi, "out_path": str(out_path)}


# ---------- 3) Example ----------
if __name__ == "__main__":
    # Intensity image (grayscale) example:
    # info_img = undistort_tof_image(
    #     "tof_intensity.png",
    #     alpha=1.0,        # 0: crop black edges; 1: keep full FOV (may show black borders)
    #     crop=False,        # crop to valid ROI when alpha=0
    #     interpolation="linear"
    # )
    # print("Saved undistorted intensity to:", info_img["out_path"])

    # Depth map example (use NEAREST!)
    info_depth = undistort_tof_depth(
        "depth_raw.png",
        alpha=1.0,
        crop=False,
        invalid_value=0
    )
    print("Saved undistorted depth to:", info_depth["out_path"])
