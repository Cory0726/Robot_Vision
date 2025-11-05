import numpy as np

import basler_rgb_cam_grab
import basler_tof_cam_grab
import cv2


if __name__ == '__main__':

    pcl = basler_tof_cam_grab.grab_one_point_cloud()
    print(pcl[:,:,2].max())
    print(pcl[:,:,2].min())
    raw_depth = basler_tof_cam_grab.pcl_to_rawdepth(pcl)
    print(raw_depth.max(), raw_depth.min())

