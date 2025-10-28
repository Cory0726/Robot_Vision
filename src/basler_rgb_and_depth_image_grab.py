from pypylon import pylon
import cv2
import lab_basler_library

def grab_one_rgb_img():
    rgb_cam_sn = "24747625"
    cam = lab_basler_library.create_basler_camera(rgb_cam_sn)
    cam.Open()
    lab_basler_library.config_rgb_camera_para(cam)

    grab_result = cam.GrabOne(1000)  # timeout: 1000 ms
    print(TypeError(grab_result))








if __name__ == "main":
    grab_one_rgb_img()