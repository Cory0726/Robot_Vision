import numpy as np
import cv2
import basler_rgb_cam_grab
import basler_tof_cam_grab
import basler_fusion_depth_rgb


if __name__ == '__main__':
    color_img = basler_rgb_cam_grab.grab_one_rgb_img()
    pcl = basler_tof_cam_grab.grab_one_point_cloud()

    pcl_color_frame = basler_fusion_depth_rgb.transform_pcl_to_color_frame(pcl)

    print(pcl_color_frame[:,:,2].max())
    print(pcl_color_frame[:,:,2].min())


