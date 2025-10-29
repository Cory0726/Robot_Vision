"""
This sample illustrates how to get images from the blaze camera using the pypylon module.
"""

from pypylon import pylon

# This is used for reshaping the image buffers.
import numpy as np

# This is used for visualization.
import cv2


# Create an instant camera object with the first camera device found.
dc = pylon.DeviceInfo()
dc.SetDeviceClass("BaslerGTC/Basler/GenTL_Producer_for_Basler_blaze_101_cameras")
camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice(dc))
camera.Open()

# Print the model name of the camera.
print("Using device ", camera.GetDeviceInfo().GetModelName())


# In the following, we demonstrate how to get and set camera parameters.
# For demonstration purposes and to avoid having to change the camera's state,
# we first get a parameter value and then set it again.

# Access OperatingMode.
operatingMode = camera.OperatingMode.Value
camera.OperatingMode.Value = operatingMode
# camera.OperatingMode.Value = 'ShortRange')
print("OperatingMode: ", camera.OperatingMode.Value)

# Access FastMode.
fastMode = camera.FastMode.Value
camera.FastMode.Value = fastMode
# camera.FastMode.Value = True)
print("FastMode: ", camera.FastMode.Value)

# Access FilterSpatial
filterSpatial = camera.FilterSpatial.Value
camera.FilterSpatial.Value = filterSpatial
# camera.FilterSpatial.Value = True)
print("FilterSpatial: ", camera.FilterSpatial.Value)

# Access FilterTemporal
filterTemporal = camera.FilterTemporal.Value
camera.FilterTemporal.Value = filterTemporal
# camera.FilterTemporal.Value = True)
print("FilterTemporal: ", camera.FilterTemporal.Value)

# Access FilterStrength
filterStrength = camera.FilterStrength.Value
camera.FilterStrength.Value = filterStrength
# FilterTemporal must be enabled before setting
# FilterStrengh is possible.
if camera.FilterTemporal.Value:
    camera.FilterStrength.Value = filterStrength
    # camera.FilterStrength.Value = 200)
print("FilterStrength: ", camera.FilterStrength.Value)

# Access OutlierRemoval
outlierRemoval = camera.OutlierRemoval.Value
camera.OutlierRemoval.Value = outlierRemoval
# camera.OutlierRemoval.Value = True)
print("OutlierRemoval: ", camera.OutlierRemoval.Value)

# Access ConfidenceThreshold
confidenceThreshold = camera.ConfidenceThreshold.Value
camera.ConfidenceThreshold.Value = confidenceThreshold
# camera.ConfidenceThreshold.Value = 20)
print("ConfidenceThreshold: ", camera.ConfidenceThreshold.Value)

# Access GammaCorrection
gammaCorrection = camera.GammaCorrection.Value
camera.GammaCorrection.Value = gammaCorrection
# camera.GammaCorrection.Value = True
print("GammaCorrection: ", camera.GammaCorrection.Value)

# Set the working range to the values displayed for the Max. Depth [mm] and
# Min. Depth [mm] parameters.
# The working range depends on the current operating mode, so
# the OperatingMode parameter must be set before adjusting DepthMax and DepthMin.
# LongRange: 0 .. 9990mm
# ShortRange: 0 .. 1498mm
depthMin = camera.DepthMin.Value
depthMax = camera.DepthMax.Value
camera.DepthMin.Value = depthMin
camera.DepthMax.Value = depthMax
# camera.DepthMin.Value = 0
# camera.DepthMax.Value = 9990
print("Min. Depth [mm]: ", camera.DepthMin.Value)
print("Max. Depth [mm]: ", camera.DepthMax.Value)

# Control pixel formats for image components.
# Range information can be sent either as a 16-bit gray value image or as
# 3D coordinates (point cloud).
# For this sample, we want to acquire 3D coordinates.
# Note: To change the format of an image component, the Component Selector parameter
# must first be set to the component
# you want to configure.
# To use 16-bit integer depth information, choose "Coord3D_C16" instead of "Coord3D_ABC32f".
camera.ComponentSelector.Value = "Range"
camera.ComponentEnable.Value = True
camera.PixelFormat.Value = "Coord3D_ABC32f"

camera.ComponentSelector.Value = "Intensity"
camera.ComponentEnable.Value = True
camera.PixelFormat.Value = "Mono16"

camera.ComponentSelector.Value = "Confidence"
camera.ComponentEnable.Value = True
camera.PixelFormat.Value = "Confidence16"

# Enable GenDC.
camera.GenDCStreamingMode.Value = "On"

print('To exit, press ESC in one of the image windows')


# Grabbing continuously (video) with minimal delay.
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

while camera.IsGrabbing():
    grabResult = camera.RetrieveResult(1000, pylon.TimeoutHandling_ThrowException)
    
    # Data grabbed successfully?
    if grabResult.GrabSucceeded():

        # Get the grab result as a PylonDataContainer, e.g., when working with 3D cameras.
        pylonDataContainer = grabResult.GetDataContainer()
        # Access data components if the component type indicates image data
        for componentIndex in range(pylonDataContainer.DataComponentCount):
            pylonDataComponent = pylonDataContainer.GetDataComponent(componentIndex)
            if pylonDataComponent.ComponentType == pylon.ComponentType_Intensity:
                # Access the component data.
                intensity = pylonDataComponent.Array
                _2d_intensity = intensity.reshape(pylonDataComponent.Height, pylonDataComponent.Width)
            elif pylonDataComponent.ComponentType == pylon.ComponentType_Range:
                pointcloud = pylonDataComponent.Array
                _3d = pointcloud.reshape(pylonDataComponent.Height, pylonDataComponent.Width, 3)
            elif pylonDataComponent.ComponentType == pylon.ComponentType_Confidence:
                confidence = pylonDataComponent.Array
                _2d_confidence = confidence.reshape(pylonDataComponent.Height, pylonDataComponent.Width)
            pylonDataComponent.Release()

        # Show the captured images as grayscale.
        # We only show the z-component of the point cloud.
        # If you choose "Coord3D_C16" as pixel format, you have to remove [:,:,2].
        # OpenCV can't show float values. We convert it for visualization to uint8.
        _3d_scaled = _3d * 255.0 / camera.DepthMax.Value
        cv2.imshow('depth', _3d_scaled[:, :, 2].astype(np.uint8))

        cv2.imshow('_2d_intensity', _2d_intensity)

        cv2.imshow('_2d_confidence', _2d_confidence)

        # Break the endless loop by pressing ESC.
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    grabResult.Release()

# Releasing the resource    
camera.StopGrabbing()

cv2.destroyAllWindows()
