from pypylon import pylon
import cv2
import lab_basler_library


def main():
    # RGB camera serial number
    RGB_CAMERA_SN = "24747625"
    
    # RGB camera initialization
    camera = lab_basler_library.create_basler_camera(RGB_CAMERA_SN)
    camera.Open()
    lab_basler_library.config_rgb_camera_para(camera)

    # Start the grabbing of images with strategy
    camera.StartGrabbing(pylon.GrabStrategy_OneByOne)
    print("Start grabbing ...")
    while camera.IsGrabbing():
        # Get the grab retrieve
        grab_retrieve = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        
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
        
            grab_retrieve.Release()
    camera.StopGrabbing()
    camera.Close()
    cv2.destroyAllWindows
    
if __name__ =="__main__":
    main()