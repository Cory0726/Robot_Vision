import basler_rgb_cam_grab
import basler_tof_cam_grab
import cv2

from src.basler_tof_cam_grab import pcl_to_rawdepth

if __name__ == '__main__':
    basler_tof_cam_grab.grab_one_point_cloud()
    img = pcl_to_rawdepth(basler_tof_cam_grab.grab_one_point_cloud())
    cv2.imwrite("test.png", img)