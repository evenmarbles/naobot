from __future__ import division, print_function, absolute_import

import numpy as np
import cv2
from cv2 import cv

from mlpy.auxiliary.datastructs import Point2D

from ..world_model.model import NaoWorldModel


class Detector(object):
    """The base detector class.

    Be ware that color based detection is much dependent on the
    current lighting and might change under different circumstances.

    Functionality for calibrating the minimum and maximum color thresholds
    for the red, green, and blue channels for color based object detection
    are provided  by setting the mode to `MODE_CALIBRATE`. In this setting,
    the detector does not need to be started.

    Use this mode to adjust the threshold when lighting in the environment
    changes.

    Parameters
    ----------
    mode : {MODE_NORMAL, MODE_CALIBRATE}
        The mode to run the detector in. When calibrating,
        the `detect` flag is being disregarded; i.e. the
        detector does not need to be started to perform
        calibration.

    Notes
    -----
    All detectors should inherit from this class.

    """
    MODE_NORMAL = 0
    MODE_CALIBRATE = 1

    def __init__(self, mode=MODE_NORMAL):
        self._detect = False
        self._mode = mode

    def change_mode(self, mode):
        """Changes teh mode of the detector.

        Parameters
        ----------
        mode : {MODE_NORMAL, MODE_CALIBRATE}
            The mode to run the detector in. When calibrating,
            the `detect` flag is being disregarded; i.e. the
            detector does not need to be started to perform
            calibration.

        Raises
        ------
        ValueError
            If the given mode is not a valid option.

        """
        if mode not in [Detector.MODE_NORMAL, Detector.MODE_CALIBRATE]:
            raise ValueError("The mode is not a valid option.")
        self._mode = mode

    def start(self):
        """Start the detector."""
        self._detect = True

    def stop(self):
        """Stop the detector."""
        self._detect = False

    def update(self, image):
        """Perform object detection.

        Parameters
        ----------
        image: CameraImage
            The camera image in which to detect the object.

        Returns
        -------
        bool
            Whether the object was detected or not.

        """
        detected = False
        if self._mode == self.MODE_CALIBRATE:
            self._calibrate(image)
            detected = True
        return detected

    def _calibrate(self, image):
        """Calibrate color for detection.

        Be ware that color based detection is much dependent on the
        current lighting and might change under different circumstances.

        This method is useful to calibrate the minimum and maximum color
        thresholds for the red, green, and blue channels for color based
        object detection. Calibration mode can be turned on by setting the
        mode to `MODE_CALIBRATE`.

        Parameters
        ----------
        image : CameraImage
            The camera image used to calibrate the thresholds.

        """
        def nothing():
            pass

        cv2.namedWindow("F1", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("F2", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("F3", cv2.WINDOW_AUTOSIZE)

        cv2.createTrackbar("min1", "F1", self._min_threshold[0], 260, nothing)
        cv2.createTrackbar("max1", "F1", self._max_threshold[0], 260, nothing)
        cv2.createTrackbar("min2", "F2", self._min_threshold[1], 260, nothing)
        cv2.createTrackbar("max2", "F2", self._max_threshold[1], 260, nothing)
        cv2.createTrackbar("min3", "F3", self._min_threshold[2], 260, nothing)
        cv2.createTrackbar("max3", "F3", self._max_threshold[2], 260, nothing)

        while True:
            img = np.copy(image.image)
            img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # Filter out colors which are out of range
            thresholded = cv2.inRange(img_hsv, self._min_threshold, self._max_threshold)

            (channel_h, channel_s, channel_v) = cv2.split(img_hsv)

            channel_h = cv2.inRange(channel_h, self._min_threshold[0], self._max_threshold[0])
            channel_s = cv2.inRange(channel_s, self._min_threshold[1], self._max_threshold[1])
            channel_v = cv2.inRange(channel_v, self._min_threshold[2], self._max_threshold[2])

            thresholded = cv2.GaussianBlur(thresholded, (9, 9), -1)

            circles = cv2.HoughCircles(thresholded, cv.CV_HOUGH_GRADIENT, 2, thresholded.shape[0] / 4, param1=100,
                                       param2=50)
            if circles is not None:
                for i in circles[0, :]:
                    center = Point2D(int(round(i[0], 0)), int(round(i[1], 0)))
                    radius = int(round(i[2]))
                    cv2.circle(img, (center.x, center.y), 3, (0, 255, 0), -1, 8, 0)
                    cv2.circle(img, (center.x, center.y), radius, (0, 255, 0), 2)

            cv2.imshow("Object", img)

            cv2.imshow("F1", channel_h)
            cv2.imshow("F2", channel_s)
            cv2.imshow("F3", channel_v)

            # If ESC key pressed
            if (cv2.waitKey(10) & 0xFF) == 27:
                break

            min1 = cv2.getTrackbarPos('min1', 'F1')
            max1 = cv2.getTrackbarPos('max1', 'F1')
            min2 = cv2.getTrackbarPos('min2', 'F2')
            max2 = cv2.getTrackbarPos('max2', 'F2')
            min3 = cv2.getTrackbarPos('min3', 'F3')
            max3 = cv2.getTrackbarPos('max3', 'F3')
            self._min_threshold = np.array([min1, min2, min3])
            self._max_threshold = np.array([max1, max2, max3])

        cv2.destroyAllWindows()


class BallDetector(Detector):
    """Color based detection of the ball.

    Detect the ball in the provided image using a color based method.

    Parameters
    ----------
    mode : {MODE_NORMAL, MODE_CALIBRATE}
        The mode to run the detector in. When calibrating,
        the `detect` flag is being disregarded; i.e. the
        detector does not need to be started to perform
        calibration.
    resolution : tuple
        The resolution of the image.

    Notes
    -----
    Using color thresholds for the color red; i.e. the color of the ball in RoboCup,
    red blobs are detected in the image. These blobs are passed to a Hough circle
    algorithm to detect red circles. The first red circle found is returned as the
    ball.

    Be ware that color based detection is much dependent on the
    current lighting and might change under different circumstances.

    Functionality for calibrating the minimum and maximum color thresholds
    for the red, green, and blue channels for detecting the ball are provided
    by setting the mode to `MODE_CALIBRATE`. In this setting, the detector
    does not need to be started.

    Use this mode to adjust the threshold when lighting in the environment
    changes.

    """

    def __init__(self, resolution, mode=Detector.MODE_NORMAL):
        super(BallDetector, self).__init__(mode)

        self._ball = NaoWorldModel().get_object("ball")
        """:type: Ball"""
        self._ball.resolution = resolution

        self._min_threshold = np.array([7, 210, 0])  # [13, 220, 0] [15, 13, 227]
        self._max_threshold = np.array([50, 245, 257])  # [35, 242, 245] [83, 247, 260]

        self._show_img = False

    def show_image(self, show=True):
        """Sets the flag to show the detected image.

        Parameters
        ----------
        show : bool, optional
            Whether to show the image or not. Default is True.

        """
        self._show_img = show

    def update(self, image):
        """Find the ball in image.

        The ball detector uses a color based method for detection.

        Parameters
        ----------
        image : CameraImage
            The camera image used.

        Returns
        -------
        bool
            Whether the ball was detected or not.

        Notes
        -----
        Using color thresholds for the color red; i.e. the color of the ball in RoboCup,
        red blobs are detected in the image. These blobs are passed to a Hough circle
        algorithm to detect red circles. The first red circle found is returned as the
        ball.

        .. todo::
            Handle multiple red blobs and determine the best fit.

        """
        detected = super(BallDetector, self).update(image)

        if not detected and self._detect:
            # Change color format from BGR to HSV
            img_hsv = cv2.cvtColor(image.image, cv2.COLOR_BGR2HSV)

            # Filter out colors which are out of range
            thresholded = cv2.inRange(img_hsv, self._min_threshold, self._max_threshold)
            # Smooth the binary image
            thresholded = cv2.GaussianBlur(thresholded, (9, 9), -1)

            circles = cv2.HoughCircles(thresholded, cv.CV_HOUGH_GRADIENT, 2, thresholded.shape[0] / 4, param1=100,
                                       param2=50)
            if circles is not None:
                for i in circles[0, :]:
                    center = Point2D(int(round(i[0], 0)), int(round(i[1], 0)))
                    radius = int(round(i[2]))
                    self._ball.resolution = (image.width, image.height)
                    self._ball.image_center = center
                    self._ball.image_radius = radius
                    self._ball.timestamp = image.timestamp
                    break  # TODO: handle multiple red blobs

                if self._show_img:
                    self._show(image.image, (self._ball.image_center.x, self._ball.image_center.y),
                               self._ball.image_radius)

                detected = True
        return detected

    # noinspection PyMethodMayBeStatic
    def _show(self, img, center, radius):
        """Show the image in a separate window.

        This shows the image in a separate window and draws
        a circle around the ball if the ball was found.

        The window can be closed by pressing the `ESC` key.

        Parameters
        ----------
        img : ndarray[Image]
            The image.
        center : tuple[int]
            The center of the ball.
        radius : float
            The radius of the ball.

        Returns
        -------
        bool
            True as long as the window is displayed.
        """
        cv2.circle(img, center, 3, (0, 255, 0), -1, 8, 0)
        cv2.circle(img, center, radius, (0, 255, 0), 2)
        cv2.imshow("Ball Found", img)

        # If ESC key pressed
        if (cv2.waitKey(10) & 255) == 27:
            cv2.destroyAllWindows()
            return True
        return False
