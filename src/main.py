import basler_rgb_cam_grab
import cv2


if __name__ == '__main__':
    org_rgb_img = basler_rgb_cam_grab.grab_one_rgb_img()
    cv2.imwrite('robot_vision_result/org_rgb_img_by_one_grab.png', org_rgb_img)

    undist_rgb_img, K = basler_rgb_cam_grab.undistort_rgb_image(org_rgb_img, 0)
    cv2.imwrite('robot_vision_result/undist_rgb_img_by_one_grab.png', undist_rgb_img)