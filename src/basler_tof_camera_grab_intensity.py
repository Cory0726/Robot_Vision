#
import os
from pypylon import pylon
import cv2
import lab_basler_library

def main():
    # ToF camera serial number
    TOF_CAMERA_SN = "24945819"
    
    # ToF camera initialization
    camera = lab_basler_library.create_basler_camera(TOF_CAMERA_SN)
    camera.Open()
    lab_basler_library.config_tof_camera_para(camera)
    lab_basler_library.config_data_component_type(camera, "Intensity")
    
    # Starts the grabbing of data with strategy
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    print("Start grabbing ...")
    while camera.IsGrabbing():
        # Get the grab retrieve
        grab_retrieve = camera.RetrieveResult(1000, pylon.TimeoutHandling_ThrowException)

        if grab_retrieve.GrabSucceeded():
            # Get the grab retrieve as data container
            data_container = grab_retrieve.GetDataContainer()
            # Get the intensity image
            data = lab_basler_library.split_container_data(data_container)
            intensity_image = data["Intensity_Image"]
            # Display
            cv2.imshow("intensity", intensity_image)
            
            grab_retrieve.Release()
            
        # Read the keyboard keyin
        key = cv2.waitKey(5) & 0xFF
        # Break the loop by pressing q
        if key == ord("q"):
            break
        
    camera.StopGrabbing()
    camera.Close()
    cv2.destroyAllWindows

if __name__ == "__main__":
    main()
    