import basler_rgb_cam_grab
import basler_tof_cam_grab
import cv2


if __name__ == '__main__':
    basler_tof_cam_grab.stream_tof_img("Intensity_Image")
