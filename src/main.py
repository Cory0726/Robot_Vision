import numpy as np
import cv2
import basler_rgb_cam_grab
import basler_tof_cam_grab
import basler_fusion_depth_rgb


if __name__ == '__main__':
    color_img = basler_rgb_cam_grab.grab_one_rgb_img()
    pcl = basler_tof_cam_grab.grab_one_point_cloud()

    pcl_color_frame = basler_fusion_depth_rgb.transform_pcl_to_color_frame(pcl)

    raw_depth_color_frame = basler_tof_cam_grab.pcl_to_rawdepth(pcl_color_frame)

    overlay_heatmap, overlay_edges = basler_fusion_depth_rgb.visualize_rgb_depth_alignment(
        raw_depth_color_frame, color_img
    )

    cv2.imshow("overlay_heatmap", overlay_heatmap)
    cv2.imshow("overlay_edges", overlay_edges)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


