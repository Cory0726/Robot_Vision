import numpy as np

import basler_rgb_cam_grab
import basler_tof_cam_grab
import cv2

from src.basler_tof_cam_grab import pcl_to_rawdepth

if __name__ == '__main__':
    basler_tof_cam_grab.grab_one_point_cloud()
    img = pcl_to_rawdepth(basler_tof_cam_grab.grab_one_point_cloud())
    img, _ = basler_tof_cam_grab.undistort_tof_depth(img)
    img =basler_tof_cam_grab.rawdepth_to_heatmap(img)
    cv2.imshow('img', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

