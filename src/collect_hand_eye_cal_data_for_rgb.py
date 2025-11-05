import os
from pypylon import pylon
import cv2
import basler_rgb_cam_grab
import read_tm_robot_modbus_data

def main():

    # RGB camera initialization
    cam = basler_rgb_cam_grab.create_rgb_cam_obj()
    cam.Open()
    basler_rgb_cam_grab.config_rgb_cam_para(cam)
    
    # Starts the grabbing of images with strategy
    cam.StartGrabbing(pylon.GrabStrategy_OneByOne)
    print("Start grabbing ...")
    while cam.IsGrabbing():
        # Get the grab retrieve
        grab_retrieve = cam.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

        if grab_retrieve.GrabSucceeded():
            bayer_image = grab_retrieve.Array
            # Convert bayer to RGB
            rgb_image = cv2.cvtColor(bayer_image, cv2.COLOR_BAYER_BG2RGB)
            cv2.imshow("RGB", rgb_image)

            # Read the keyboard keyin
            key = cv2.waitKey(5) & 0xFF
            # Break the loop by pressing q
            if key == ord("q"):
                break
            # Save the current image by pressing s
            elif key == ord("s"):
                file_number = 0
                file_path = f"halcon_calibration_img/img{file_number:02d}.png"
                while os.path.exists(file_path):
                    file_number += 1
                    file_path = f"halcon_calibration_img/img{file_number:02d}.png"
                # Convert color to gray
                gray_img = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
                cv2.imwrite(file_path, gray_img)
                print(f"Saved: {file_path}")
                # Save the TM robot flange pose (.dat)
                read_tm_robot_modbus_data.save_TM_robot_flange_pose("TM5x_700")
                
            grab_retrieve.Release()
    cam.StopGrabbing()
    cam.Close()
    cv2.destroyAllWindows


if __name__ == "__main__":
    main()