"""
This sample demonstrates how to calibrate a system consisting of a Basler GigE color camera
and a Basler blaze camera.

The project consists of two files: one for calibration and one for fusion. In the first step,
the calibration file must be used to determine the relative position of the cameras
to each other.

To do this, take several images of a chessboard by pressing the S key. After that, start the
calibration by pressing the C key. For the calibration, 10 to 15 images of different areas
of the chessboard should be taken, covering the whole field of view. A suitable chessboard is
provided with this sample. This should be printed as DIN A3 (297 x 420 mm), so that the squares
have a side length of 40 mm each. If the side length of the squares in your printout is different,
adjust the specified size in the calibration program. Calibration only needs to be performed
once or whenever the configuration of the system is changed. This is the case when you change,
e.g., the position of a camera.
"""

import os
import platform
import traceback

# This is used for reshaping the image buffers.
import numpy as np

# This is used for visualization and debayering.
import cv2

# Use of Harvester to access the camera.
# For more information regarding Harvester, visit the github page:
# https://github.com/genicam/harvesters
from harvesters.core import Harvester


# Chessboard size
chessboard_rows = 8
chessboard_cols = 5
# For a DIN A3 board in this sample, each square is 40 mm.
field_size = 40  # mm


BAYER_FORMATS = {"BayerGR8": cv2.COLOR_BayerGR2GRAY,
                 "BayerRG8": cv2.COLOR_BayerRG2GRAY,
                 "BayerBG8": cv2.COLOR_BayerBG2GRAY,
                 "BayerGB8": cv2.COLOR_BayerGB2GRAY}


def find_producer(name):
    """ Helper for the GenTL producers from the environment path.
    """
    paths = os.environ['GENICAM_GENTL64_PATH'].split(os.pathsep)

    if platform.system() == "Linux":
        paths.append('/opt/pylon/lib/gentlproducer/gtl/')

    for path in paths:
        path += os.path.sep + name
        if os.path.exists(path):
            return path
    return ""


class Calibration:
    """
    Encapsulates the full stereo calibration workflow.
    """
    def __init__(self):
        # Create Harvester instances.
        self.h = Harvester()

        # Location of the Basler blaze GenTL producer.
        if platform.system() == "Windows" or platform.system() == "Linux":
            path_to_blaze_cti = find_producer("ProducerBaslerBlazePylon.cti")
            path_to_gev_cti = find_producer("ProducerGEV.cti")
        else:
            print(f"{platform.system()} is not supported")
            assert False

        # Add producer to Harvester.
        assert os.path.exists(path_to_blaze_cti)
        assert os.path.exists(path_to_gev_cti)

        self.h.add_file(path_to_blaze_cti)
        self.h.add_file(path_to_gev_cti)

        # Update device list.
        self.h.update()

        # Print device list.
        print(self.h.device_info_list)

    def setup_blaze(self):
        """
        Connect and configure the Basler blaze camera (3D ToF).
        """
        dev_info = next(
            (d for d in self.h.device_info_list if str(d.model).startswith('blaze')), None)
        if dev_info is not None:
            self.ia_blaze = self.h.create({"model":dev_info.model, "serial_number":dev_info.serial_number})
            print("Connected to blaze camera: {}".format(
                self.ia_blaze.remote_device.node_map.DeviceSerialNumber.value))
        else:
            print("No blaze camera found.")
            raise RuntimeError

        # Disable depth data.
        self.ia_blaze.remote_device.node_map.ComponentSelector.value = "Range"
        self.ia_blaze.remote_device.node_map.ComponentEnable.value = False
        self.ia_blaze.remote_device.node_map.PixelFormat.value = "Coord3D_C16"

        # Enable the intensity image .
        self.ia_blaze.remote_device.node_map.ComponentSelector.value = "Intensity"
        self.ia_blaze.remote_device.node_map.ComponentEnable.value = True
        self.ia_blaze.remote_device.node_map.PixelFormat.value = "Mono16"

        # Disable the confidence map.
        self.ia_blaze.remote_device.node_map.ComponentSelector.value = "Confidence"
        self.ia_blaze.remote_device.node_map.ComponentEnable.value = False
        self.ia_blaze.remote_device.node_map.PixelFormat.value = "Confidence16"

        # Reduce exposure time to avoid overexposure at close range.
        self.ia_blaze.remote_device.node_map.ExposureTime.value = 250

        # Switch off gamma correction for accurate corner detection.
        self.ia_blaze.remote_device.node_map.GammaCorrection.value = False

        # Configure the camera for software triggering.
        self.ia_blaze.remote_device.node_map.TriggerMode.value = "On"
        self.ia_blaze.remote_device.node_map.TriggerSource.value = "Software"

        # Disable GenDC. This mode is currently not supported by the python GenICam module.
        self.ia_blaze.remote_device.node_map.GenDCStreamingMode.value = "Off"

        # Start image acquisition.
        self.ia_blaze.start()

    def setup_2Dcamera(self):
        """
        Connect and configure the Basler 2D GigE color camera.
        """
        # Connect to the first available 2D camera. Ignore blaze cameras, which will
        # be enumerated as well.
        dev_info = next(
            (d for d in self.h.device_info_list if 'blaze' not in d.model), None)
        if dev_info is not None:
            self.ia_gev = self.h.create({"serial_number":dev_info.serial_number})
            print("Connected to ace camera: {}".format(
                dev_info.serial_number))
        else:
            print("No 2D camera found.")
            raise RuntimeError

        # Figure out which 8-bit Bayer pixel format the camera supports.
        # If the camera supports an 8-bit Bayer format, enable the format.
        # Otherwise, exit the program.
        bayer_pattern = next((pf for pf in self.ia_gev.remote_device.node_map.PixelFormat.symbolics
                              if pf in BAYER_FORMATS), None)
        if bayer_pattern is not None:
            self.ia_gev.remote_device.node_map.PixelFormat.value = bayer_pattern
        else:
            print("The camera does not provide Bayer pattern-encoded 8-bit color images.")
            raise RuntimeError

        # Configure the camera for software triggering.
        # Each software trigger will start the acquisition of one single frame.
        self.ia_gev.remote_device.node_map.AcquisitionMode.value = "Continuous"
        self.ia_gev.remote_device.node_map.TriggerSelector.value = "FrameStart"
        self.ia_gev.remote_device.node_map.TriggerMode.value = "On"
        self.ia_gev.remote_device.node_map.TriggerSource.value = "Software"

        # Start image acquisition.
        self.ia_gev.start()

    def close_blaze(self):
        """
        Stop acquisition and disconnect from the blaze camera.
        """
        self.ia_blaze.stop()
        self.ia_blaze.remote_device.node_map.TriggerMode.value = "Off"

        # Disconnect from camera.
        self.ia_blaze.destroy()

    def close_2DCamera(self):
        """
        Stop acquisition and disconnect from the 2D camera.
        """
        self.ia_gev.stop()
        self.ia_gev.remote_device.node_map.TriggerMode.value = "Off"

        # Disconnect from camera.
        self.ia_gev.destroy()

    def close_harvesters(self):
        """
        Release producer files and reset Harvester.
        """
        self.h.reset()

    def get_image_blaze(self):
        """
        Grab a single blaze Intensity frame and convert to 8-bit grayscale.

        Returns:
            np.ndarray (H, W), dtype=uint8: Grayscale image suitable for corner detection.
        """
        with self.ia_blaze.fetch() as buffer:
            # Warning: The buffer is only valid in the with statement and will be destroyed
            # when you leave the scope.
            # If you want to use the buffers outside of the with scope, you have to use np.copy()
            # to make a deep copy of the image.

            # Create an alias of the image components:
            intensity = buffer.payload.components[0]

            # Reshape the intensity image into a 2D array:
            _2d_intensity = intensity.data.reshape(
                intensity.height, intensity.width)
            gray = _2d_intensity/256.0
            gray = np.uint8(gray)

            return gray

    def get_image_2DCamera(self):
        """
        Grab a single 2D color camera frame and convert to 8-bit grayscale.

        Returns:
            np.ndarray (H, W), dtype=uint8: Grayscale image for chessboard corner detection.
        """
        with self.ia_gev.fetch() as buffer:
            # Warning: The buffer is only valid in the with statement and will be destroyed
            # when you leave the scope.
            # If you want to use the buffers outside of the with scope, you have to use np.copy()
            # to make a deep copy of the image.

            # Create an alias of the image component:
            image = buffer.payload.components[0]

            # Reshape the image into a 2D array:
            _2d = image.data.reshape(image.height, image.width)

            # Debayer the image to a grayscale image.
            gray = cv2.cvtColor(_2d, BAYER_FORMATS[image.data_format])

            return gray

    def locate_chessboard_corners(self, gray):
        """
        Find and refine chessboard corners in a grayscale image.

        Args:
            gray (np.ndarray): Grayscale image.

        Returns:
            (bool, np.ndarray or None): (found_flag, refined_corners)
        """
        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_COUNT, 40, 0.001)
        ret, corners = cv2.findChessboardCorners(
            gray, (chessboard_rows, chessboard_cols))

        if ret == True:
            corners = cv2.cornerSubPix(
                gray, corners, (7, 7), (-1, -1), criteria)

        return ret, corners

    def color_calibration(self, obj_points, color_points, color_shape):
        """
        Calibrate the intrinsic parameters of the 2D color camera.

        Args:
            obj_points (list[np.ndarray]): 3D object points for each view (Z=0 plane).
            color_points (list[np.ndarray]): Detected 2D corner points per view.
            color_shape (tuple): Image size (width, height) as expected by OpenCV.

        Returns:
            (np.ndarray, np.ndarray): (camera_matrix, distortion_coeffs)
        """
        rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, color_points,
                                                           color_shape, None, None, 
                                                           flags= (cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3))
        print('Reprojection error color camera:', rms)

        return mtx, dist

    def blaze_calibration(self):
        """
        Retrieve blaze intrinsics from the device nodemap (treated as known).

        Returns:
            (np.ndarray, np.ndarray): (camera_matrix, zero_distortion)
        """
        f = self.ia_blaze.remote_device.node_map.Scan3dFocalLength.value
        cx = self.ia_blaze.remote_device.node_map.Scan3dPrincipalPointU.value
        cy = self.ia_blaze.remote_device.node_map.Scan3dPrincipalPointV.value

        mtx = np.zeros((3, 3), np.float32)
        mtx[0, 0] = f
        mtx[0, 2] = cx
        mtx[1, 1] = f
        mtx[1, 2] = cy
        mtx[2, 2] = 1

        dist = np.zeros((1, 5), np.float32)

        return mtx, dist

    def stereo_calibration(self, obj_points, color_points, color_camera_matrix,
                           color_dist, color_shape, blaze_points, blaze_camera_matrix, blaze_dist):
        """
        Run stereo calibration (blaze - 2D color) to estimate rotation/translation.
        Save the results to an XML file alongside this script.
        Args:
            obj_points: Shared 3D object points
            color_points: 2D points in the color camera.
            color_camera_matrix, color_dist: Intrinsic/distortion for the color camera.
            color_shape: Image size of the color camera (width, height).
            blaze_points: 2D points in the blaze intensity image.
            blaze_camera_matrix, blaze_dist: Intrinsic/distortion for blaze.

        Persists:
            - colorCameraMatrix, colorDistortion
            - blazeCameraMatrix, blazeDistortion
            - rotation (R), translation (T)

        """
        rotation = np.zeros((3, 3), np.float32)
        translation = np.zeros((1, 3), np.float32)

        # Calculate transformation between the two cameras.
        rms, new_mtxL, distL, new_mtxR, distR, rot, trns, emat, fmat = cv2.stereoCalibrate(
                    obj_points, \
                    blaze_points, \
                    color_points, \
                    blaze_camera_matrix, \
                    blaze_dist, \
                    color_camera_matrix, \
                    color_dist, \
                    color_shape)
        print('Reprojection error stereo setup:', rms)

        # Write calibration.
        # Persist calibration results to an XML file (OpenCV FileStorage).
        dirname = os.path.dirname(__file__)
        filename = "calibration_" + str(self.ia_blaze.remote_device.node_map.DeviceSerialNumber.value) + \
            "_" + str(self.ia_gev.remote_device.node_map.DeviceID.value) + ".xml"
        path = os.path.join(dirname, filename)
        cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
        cv_file.write("colorCameraMatrix", color_camera_matrix)
        cv_file.write("colorDistortion", color_dist)
        cv_file.write("blazeCameraMatrix", blaze_camera_matrix)
        cv_file.write("blazeDistortion", blaze_dist)
        cv_file.write("rotation", rot)
        cv_file.write("translation", trns)
        cv_file.release()
        print("Wrote calibration to", path)

    def run(self):
        """
        - Sets up both camera.
        - Shows live windows.
        - Main interactive loop:
            - Press 's' to sample chessboard images from both cameras (if detectable).
            - Press 'c' to run calibration when enough samples exist.
            - Press 'q' (or Esc) to quit.
        """
        # Set up the cameras.
        self.setup_blaze()
        self.setup_2Dcamera()

        print('')
        print('Calibration of the camera system')
        print('  - Press "s" in one of the image windows to capture images of the chessboard for calibration')
        print('  - Press "c" in one of the image windows to perform the calibration with the captured images')
        print('  - Press "q" in one of the image windows to exit')
        print('')
        print('For the calibration, about 10 to 15 images should be taken from different positions covering the entire field of view.')
        print('')

        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0).
        objp = np.zeros((chessboard_rows*chessboard_cols, 3), np.float32)
        objp[:, :2] = np.mgrid[0:chessboard_rows,
                               0:chessboard_cols].T.reshape(-1, 2)
        objp = objp*field_size

        # Arrays to store object points and image points of all the images.
        obj_points = []
        blaze_points = []
        color_points = []

        # Grab the images.
        cnt = 0
        while True:
            # To optimize bandwidth usage, the color camera is triggered first to
            # allow it to already transfer image data while the blaze camera is still internally
            # processing the acquired raw data.
            self.ia_gev.remote_device.node_map.TriggerSoftware.execute()
            self.ia_blaze.remote_device.node_map.TriggerSoftware.execute()

            blaze_img = self.get_image_blaze()
            color_img = self.get_image_2DCamera()

            # Show the captured images.
            cv2.imshow('blaze image', blaze_img)
            cv2.imshow('color image', color_img)

            k = cv2.waitKey(5) & 0xFF
            if k == 27 or k == ord('q'):

                # Cancel and close program.
                break

            elif k == ord('s'):

                # Detect chessboard corner points.
                blaze_found, blaze_corners = self.locate_chessboard_corners(
                    blaze_img)
                color_found, color_corners = self.locate_chessboard_corners(
                    color_img)

                # Saving the detected chessboard corner points.
                if blaze_found == True and color_found == True:
                    print('Added images from position', cnt + 1)
                    obj_points.append(objp)
                    blaze_points.append(blaze_corners)
                    color_points.append(color_corners)
                    cv2.imwrite("blaze_" + str(cnt) + ".png", blaze_img)
                    cv2.imwrite("color_" + str(cnt) + ".png", color_img)
                    cnt = cnt + 1
                else:
                    print(
                        'Chessboard was not found. Please make sure that the chessboard is completely visible in both camera images.')

            elif k == ord('c'):

                # Perform calibration using the previously captured images.
                if len(obj_points) > 1:
                    print('Calibration')
                    color_camera_matrix, color_dist = self.color_calibration(obj_points,
                                                                             color_points, color_img.shape[::-1])
                    blaze_camera_matrix, blaze_dist = self.blaze_calibration()
                    self.stereo_calibration(obj_points, color_points, color_camera_matrix,
                                            color_dist, color_img.shape[::-1], blaze_points, blaze_camera_matrix, blaze_dist)
                    print('Done')
                    break
                else:
                    print('Not enough images for calibration available!')

        # Close the camera and release the producers.
        self.close_blaze()
        self.close_2DCamera()
        self.close_harvesters()


if __name__ == "__main__":
    """ Run the sample.
    """
    Sample = Calibration()
    try:
        Sample.run()
    except Exception:
        traceback.print_exc()
        Sample.close_harvesters()
