import os
from pypylon import pylon
import cv2
import lab_basler_library
import read_tm_robot_modbus_data

def main():
    # ToF camera serial number
    TOF_CAMERA_SN = "24945819"
    # File configuration for saving
    FILE_DIR = "calibration_img/"
    FILE_NAME = "img"
    EXTENSION = ".png"
    
    # ToF camera initialization
    camera = lab_basler_library.create_basler_camera(TOF_CAMERA_SN)
    camera.Open()
    lab_basler_library.config_tof_camera_para(camera)
    lab_basler_library.config_data_component_type(camera, "Confidence_Map")
    
    # Starts the grabbing of images with strategy
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    print("Start grabbing ...")
    while camera.IsGrabbing():
        # Get the grab retrieve
        grab_retrieve = camera.RetrieveResult(1000, pylon.TimeoutHandling_ThrowException)

        if grab_retrieve.GrabSucceeded():
            # Get the grab result as data container
            data_container = grab_retrieve.GetDataContainer()
            # Get the confidence map
            data = lab_basler_library.split_container_data(data_container)
            confidence_map = data["Confidence_Map"]
            # Display
            cv2.imshow("Confidence_Map", confidence_map)
            
            # Read the keyboard keyin
            key = cv2.waitKey(5) & 0xFF
            # Break the loop by pressing q
            if key == ord("q"):
                break
            # Save the current image by pressing s
            elif key == ord("s"):
                file_number = 0
                file_path = f"{FILE_DIR}{FILE_NAME}{file_number:02d}{EXTENSION}"
                while os.path.exists(file_path):
                    file_number += 1
                    file_path = f"{FILE_DIR}{FILE_NAME}{file_number:02d}{EXTENSION}"
                cv2.imwrite(file_path, confidence_map)
                print(f"Saved: {file_path}")
                # Save the TM robot flange pose (.dat)
                read_tm_robot_modbus_data.save_TM_robot_flange_pose("TM5x_700")
                
            grab_retrieve.Release()
    camera.StopGrabbing()
    camera.Close()
    cv2.destroyAllWindows


if __name__ == "__main__":
    main()