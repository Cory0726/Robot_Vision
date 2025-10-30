import cv2
import numpy as np
from pathlib import Path

def halcon_to_opencv_intrinsics(
    Sx_um=8.30179, Sy_um=8.3,             # pixel size [micrometers]
    f_mm=7.09725,                         # focal length [mm]
    Cx_px=635.69, Cy_px=535.704,          # principal point [pixels]
    K1=5067.98, K2=8.93518e6, K3=-6.00845e10,  # radial distortion [1/m^2, 1/m^4, 1/m^6]
    P1=-0.128108, P2=0.130449,                 # tangential distortion [1/m]
):
    """
    Convert HALCON-style camera parameters into OpenCV-style intrinsic matrix and distortion coefficients.
    HALCON expresses distortion in metric units; OpenCV expects normalized coordinates.
    """

    # --- Unit conversion to meters ---
    f_m  = f_mm / 1000.0     # mm → m
    Sx_m = Sx_um * 1e-6      # μm → m
    Sy_m = Sy_um * 1e-6

    # --- Compute focal lengths in pixels ---
    fx = f_m / Sx_m
    fy = f_m / Sy_m
    cx = Cx_px
    cy = Cy_px

    # --- Camera intrinsic matrix ---
    K = np.array([[fx, 0,  cx],
                  [0,  fy, cy],
                  [0,   0,  1]], dtype=np.float64)

    # --- Convert HALCON distortion coefficients to OpenCV format ---
    # (since HALCON uses metric units, we multiply by f powers)
    k1 = K1 * (f_m**2)
    k2 = K2 * (f_m**4)
    k3 = K3 * (f_m**6)
    p1 = P1 * (f_m**2)
    p2 = P2 * (f_m**2)

    dist = np.array([k1, k2, p1, p2, k3], dtype=np.float64)
    return K, dist

def undistort_rgb_image(
    img_path,
    out_path=None,
    img_size=(1280, 1024)
):
    """
    Read an RGB image, apply lens undistortion using OpenCV, and save the corrected image.
    """

    # 1) Build OpenCV intrinsics and distortion coefficients
    K, dist = halcon_to_opencv_intrinsics()

    # 2) Load image
    img = cv2.imread(str(img_path), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise FileNotFoundError(f"Cannot read image: {img_path}")
    h, w = img.shape[:2]

    # 3) Compute new optimal camera matrix (alpha controls cropping)
    # The parameter 'alpha' controls the trade-off between cropping and keeping the full field of view:
    # - alpha = 0 → All black borders are removed (cropped image, smaller FOV)
    # - alpha = 1 → Keep all pixels, including black borders (larger FOV)
    # - Intermediate values (0 < alpha < 1) balance between the two.
    newK, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=1)

    # 4) Undistort image
    map1, map2 = cv2.initUndistortRectifyMap(K, dist, R=None, newCameraMatrix=newK,
                                              size=(w, h), m1type=cv2.CV_32FC1)
    undist = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)

    # Alternatively, use remapping for faster processing:
    # undist = cv2.undistort(img, K, dist, None, newK)

    # 5) Crop the image based on ROI (optional)
    # x, y, rw, rh = roi
    # if rw > 0 and rh > 0:
    #     undist = undist[y:y+rh, x:x+rw]

    # 6) Save the undistorted image
    if out_path is None:
        out_path = Path(img_path).with_name(Path(img_path).stem + "_undistorted.png")
    cv2.imwrite(str(out_path), undist)

    # Return useful info for debugging
    return {
        "cameraMatrix": K,
        "distCoeffs": dist,
        "newCameraMatrix": newK,
        "roi": roi,
        "out_path": str(out_path)
    }

if __name__ == "__main__":
    # Example: Undistort RGB image from calibration results
    info = undistort_rgb_image("test.png")
    print("cameraMatrix =\n", info["cameraMatrix"])
    print("distCoeffs   = ", info["distCoeffs"].ravel())
    print("saved to     = ", info["out_path"])
