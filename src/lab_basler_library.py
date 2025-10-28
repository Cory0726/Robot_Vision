import numpy as np
from pypylon import pylon
import cv2

def list_basler_devices() -> None:
    """ 
    List all devices which connected to the computer.
    """
    # Build the transport layer factory
    tl_factory = pylon.TlFactory.GetInstance()
    # List the devices
    devices = tl_factory.EnumerateDevices()
    for i, dec in enumerate(devices):
        print(f"[{i}] {dec.GetModelName()} - {dec.GetDeviceClass()}"
            + f"- {dec.GetFullName()} - {dec.GetSerialNumber()}")

def create_basler_camera(serial_number: str) -> pylon.InstantCamera:
    """
    Create a Basler camera instance by serial number.

    Args:
        serial_number (str): Basler camera's serial number.

    Returns:
        pylon.InstantCamera: Basler camera instance
    """
    # Get the transport layer factory
    tl_factory = pylon.TlFactory.GetInstance()
    # Set the device information
    device = pylon.DeviceInfo()
    device.SetSerialNumber(serial_number)
    # Create the camera
    camera = pylon.InstantCamera(tl_factory.CreateDevice(device))
    return camera

#
# For ToF camera (Basler blaze-101)
#

def config_data_component_type(camera: pylon.InstantCamera, data_type: str) -> None:
    """
    Configurate ToF camera (Basler blaze-101) output data component type

    Args:
        camera (pylon.InstantCamera): A ToF camera instance
        data_type (str): Point_Cloud or Intensity or Confidence_Map
    """
    #Image component selector
    if data_type == "Intensity":
        # Close 3d point cloud image
        camera.GetNodeMap().GetNode("ComponentSelector").SetValue("Range")
        camera.GetNodeMap().GetNode("ComponentEnable").SetValue(False)
        camera.GetNodeMap().GetNode("PixelFormat").SetValue("Coord3D_ABC32f")
        # Open intensity image
        camera.GetNodeMap().GetNode("ComponentSelector").SetValue("Intensity")
        camera.GetNodeMap().GetNode("ComponentEnable").SetValue(True)
        camera.GetNodeMap().GetNode("PixelFormat").SetValue("Mono16")
        # Close confidence map
        camera.GetNodeMap().GetNode("ComponentSelector").SetValue("Confidence")
        camera.GetNodeMap().GetNode("ComponentEnable").SetValue(False)
        camera.GetNodeMap().GetNode("PixelFormat").SetValue("Confidence16")
        print("Image selector: Intensity")
    elif data_type == "Point_Cloud":
        # Open 3d point cloud image
        camera.GetNodeMap().GetNode("ComponentSelector").SetValue("Range")
        camera.GetNodeMap().GetNode("ComponentEnable").SetValue(True)
        camera.GetNodeMap().GetNode("PixelFormat").SetValue("Coord3D_ABC32f")
        # Close intensity image
        camera.GetNodeMap().GetNode("ComponentSelector").SetValue("Intensity")
        camera.GetNodeMap().GetNode("ComponentEnable").SetValue(False)
        camera.GetNodeMap().GetNode("PixelFormat").SetValue("Mono16")
        # Close confidence map
        camera.GetNodeMap().GetNode("ComponentSelector").SetValue("Confidence")
        camera.GetNodeMap().GetNode("ComponentEnable").SetValue(False)
        camera.GetNodeMap().GetNode("PixelFormat").SetValue("Confidence16")
        print("Image selector: Point cloud")
    elif data_type =="Confidence_Map":
        # Close 3d point cloud image
        camera.GetNodeMap().GetNode("ComponentSelector").SetValue("Range")
        camera.GetNodeMap().GetNode("ComponentEnable").SetValue(False)
        camera.GetNodeMap().GetNode("PixelFormat").SetValue("Coord3D_ABC32f")
        # Close intensity image
        camera.GetNodeMap().GetNode("ComponentSelector").SetValue("Intensity")
        camera.GetNodeMap().GetNode("ComponentEnable").SetValue(False)
        camera.GetNodeMap().GetNode("PixelFormat").SetValue("Mono16")
        # Open confidence map
        camera.GetNodeMap().GetNode("ComponentSelector").SetValue("Confidence")
        camera.GetNodeMap().GetNode("ComponentEnable").SetValue(True)
        camera.GetNodeMap().GetNode("PixelFormat").SetValue("Confidence16")
        print("Image selector: Confidence Map")
    else:
        print("Wrong data type input of function config_tof_camera_para")

def config_tof_camera_para(camera: pylon.InstantCamera) -> None:
    """
    Configurate ToF camera (Basler blaze-101) parameter after opening the camera.

    Args:
        camera (pylon.InstantCamera): A ToF camera instance
    """
    print("ToF camera information:")
    # Operating mode
    # ShortRange: 0 - 1498 mm
    # LongRange: 0 - 9990 mm
    camera.OperatingMode.Value = "ShortRange"
    print(f"Operating mode: {camera.OperatingMode.Value}")
    # Fast mode
    camera.FastMode.Value = True
    # Filter spatial
    camera.FilterSpatial.Value = True
    # Filter temporal
    camera.FilterTemporal.Value = False
    # Filter temporal strength
    if camera.FilterTemporal.Value:
        camera.FilterStrength.Value = 200
    # Outlier removal
    camera.OutlierRemoval.Value = True
    # Confidence Threshold (0 - 65536)
    camera.ConfidenceThreshold.Value = 304
    print(f"Confidence threshold: {camera.ConfidenceThreshold.Value}")
    # Gamma correction
    camera.GammaCorrection.Value = True
    # Max depth (mm)
    camera.DepthMax.Value = 1400
    print(f"Depth max: {camera.DepthMax.Value}")
    # Mim depth (mm)
    camera.DepthMin.Value = 100
    print(f"Depth min: {camera.DepthMin.Value}")    
    # GenDC (Generic Data Container) is used to transmit multiple types of image data,such as depth,
    # intensity, and confidence, in a single, structured data stream, making it
    # ideal for 3D and multi-modal imaging applications.
    camera.GenDCStreamingMode.Value = "Off"

def split_container_data(container) -> dict:
    """
    Split the data component from the grab retrieve data container

    Args:
        container (any): A grab retrieve as data container

    Returns:
        dict: data_dict{Intensity_Image, Confidence_Map, Point_Cloud}
    """
    data_dict = {
        "Intensity_Image": None,
        "Confidence_Map": None,
        "Point_Cloud": None
    }
    for i in range(container.DataComponentCount):
        data_component = container.GetDataComponent(i)
        if data_component.ComponentType == pylon.ComponentType_Intensity:
            data_dict["Intensity_Image"] = data_component.Array
        elif data_component.ComponentType == pylon.ComponentType_Confidence:
            data_dict["Confidence_Map"] = data_component.Array
        elif data_component.ComponentType == pylon.ComponentType_Range:
            data_dict["Point_Cloud"] = data_component.Array.reshape(data_component.Height, data_component.Width, 3)
        data_component.Release()
    return data_dict

def get_depth_image(point_cloud):
    """
    Convert 3D point cloud data to 2D depth image

    Args:
        point_cloud (any): Point cloud data

    Returns:
        any: 2D depth image
    """
    z_data = point_cloud[:,:,2]
    # Normalization
    z_norm = cv2.normalize(z_data, None, 0, 255,cv2.NORM_MINMAX)
    depth_image = z_norm.astype(np.uint8)
    return depth_image

#
# For RGB camera (acA1300-75gc)
#
def config_rgb_camera_para(camera: pylon.InstantCamera) -> None:
    """
    Configurate RGB camera (acA1300-75gc) parameter after opening the camera.

    Args:
        camera (pylon.InstantCamera): A RGB camera instance
    """
    # Width and height
    camera.Width.Value = 1280
    camera.Height.Value = 1024
    # Pixel format
    camera.PixelFormat.Value = "BayerBG8"
    # Exposure time (Abs) [us]
    camera.ExposureTimeAbs.Value = 7500
    # Exposure auto
    camera.ExposureAuto.Value = "Off"
    # Gain (Raw)
    camera.GainSelector.Value = "All"
    camera.GainRaw.Value = 136
    # Gain auto
    camera.GainAuto.Value = "Off"
    # Balance white auto
    camera.BalanceWhiteAuto.Value = "Off"
