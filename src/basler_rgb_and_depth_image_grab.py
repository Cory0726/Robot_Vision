import numpy as np
from pypylon import pylon
import cv2
import lab_basler_library

def grab_one_rgb_img():
    # Init rgb camera
    rgb_cam_sn = "24747625"
    rgb_cam = lab_basler_library.create_basler_camera(rgb_cam_sn)
    rgb_cam.Open()
    lab_basler_library.config_rgb_camera_para(rgb_cam)
    # Grab one rgb image
    grab_result = rgb_cam.GrabOne(100)  # timeout: 100 ms
    if grab_result.GrabSucceeded():
        bayer_image = grab_result.Array
        rgb_image = cv2.cvtColor(bayer_image, cv2.COLOR_BAYER_BG2RGB)  # Convert bayer to RGB
    grab_result.Release()
    rgb_cam.Close()
    return rgb_image

def grab_one_depth(data_type: str):
    # Init tof camera
    tof_cam_sn = "24945819"
    tof_cam = lab_basler_library.create_basler_camera(tof_cam_sn)
    tof_cam.Open()
    lab_basler_library.config_tof_camera_para(tof_cam)
    lab_basler_library.config_data_component_type(tof_cam, "Point_Cloud")
    # Grab point cloud data
    grab_result = tof_cam.GrabOne(1000)  # timeout: 1s
    assert grab_result.GrabSucceeded(), "Failed to grab depth data"
    point_cloud = lab_basler_library.split_container_data((grab_result.GetDataContainer()))["Point_Cloud"]
    z_data = point_cloud[:,:,2]
    # Data process
    if data_type == "raw_depth":
        return z_data
    elif data_type == "depth_img":
        gray_img = (z_data / tof_cam.DepthMax.Value * 255.0).astype(np.uint8)
        heatmap = cv2.applyColorMap(255 - gray_img, cv2.COLORMAP_TURBO)
        # heatmap = cv2.applyColorMap(255 - gray_img, cv2.COLORMAP_JET)
        return heatmap

if __name__ == "__main__":
    img = grab_one_depth("depth_img")
    cv2.imshow("Depth image", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()