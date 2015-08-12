from __future__ import division, print_function, absolute_import

import sys
import traceback
import numpy as np

# noinspection PyPackageRequirements
from PIL import Image

import vision_definitions
from naoqi import ALProxy

from mlpy.modules import Module
from mlpy.constants import micro

from .detectors import BallDetector


kTopCamera = 0
kBottomCamera = 1


class CameraImage(object):
    """The camera image.

    Parameters
    ----------
    img : ndarray[Image]
        The raw image data.
    width : int
        The width of the image.
    height : int
        The height of the image.
    timestamp : float
        The time the image was captured.
    camera_id : {kTopCamera, kBottomCamera}
        Whether the bottom or the top camera captured
        the image.

    Attributes
    ----------
    image : ndarray[Image]
        The raw image data.
    width : int
        The width of the image.
    height : int
        The height of the image.
    timestamp : float
        The time the image was captured.
    camera_id : {kTopCamera, kBottomCamera}
        Whether the bottom or the top camera captured
        the image.

    """

    def __init__(self, img, width, height, timestamp, camera_id):
        self.image = img
        self.width = width
        self.height = height
        self.timestamp = timestamp
        self.camera_id = camera_id


class ImageProcessor(Module):
    """The image processor.

    Processes the images provided by the bottom and top cameras and
    detects relevant objects.

    Parameters
    ----------
    pip : str
        The IP of the agent for which to process the camera images.
    pport : int
        The port of the agent for which to process the camera images.

    Notes
    -----
    For object detection, the bottom camera *always* takes precedence.
    Currently the image processor only handles ball detection.

    """
    def __init__(self, pip, pport):
        super(ImageProcessor, self).__init__(self._generate_id(pip, pport))

        self._cameraProxy = None
        self._subscriber_btm_id = None
        self._subscriber_top_id = None

        self._resolution = vision_definitions.kQVGA
        self._ball_detector = BallDetector(self._resolution)

        self._initialize(pip, pport)

    def reset(self, t, **kwargs):
        """Reset the image processor.

        Ensure that the subscriptions to the cameras are valid.

        Parameters
        ----------
        t : float
            The current time (sec)
        kwargs : dict, optional
            Non-positional parameters, optional.

        """
        super(ImageProcessor, self).reset(t, **kwargs)
        self._initialize(self.mid.split(':')[1], int(self.mid.split(':')[2]))

    def enter(self, t):
        """Perform preliminary setup.

        Parameters
        ----------
        t : float
            The current time (sec).

        """
        super(ImageProcessor, self).enter(t)
        self._ball_detector.start()

    def update(self, dt):
        """Update the image processor.

        Capture images from the camera and detect objects.
        The bottom camera takes precedence.

        Parameters
        ----------
        dt : float
            The elapsed time (sec)

        """
        super(ImageProcessor, self).update(dt)

        image_btm = self._get_image(kBottomCamera)
        if not self._ball_detector.update(image_btm):
            image_top = self._get_image(kTopCamera)
            self._ball_detector.update(image_top)

    def exit(self):
        """Exit the image processor.

        Stop the detectors and unsubscribe from the cameras.

        """
        super(ImageProcessor, self).exit()

        self._ball_detector.stop()
        try:
            self._cameraProxy.ping()
            if self._subscriber_top_id is not None:
                self._cameraProxy.unsubscribe(self._subscriber_top_id)
            if self._subscriber_btm_id is not None:
                self._cameraProxy.unsubscribe(self._subscriber_btm_id)
        except:
            pass

    def get_resolution(self):
        """Returns the image resolution.

        Returns
        -------
        tuple[int]
            The resolution of the image.

        """
        resolution = None
        if self._resolution == vision_definitions.kQQVGA:
            resolution = (160, 120)
        if self._resolution == vision_definitions.kQVGA:
            resolution = (320, 240)
        if self._resolution == vision_definitions.kVGA:
            resolution = (640, 480)
        if self._resolution == vision_definitions.k4VGA or self._resolution == vision_definitions.k960p:
            resolution = (1280, 960)
        return resolution

    def _initialize(self, pip, pport):
        """Initializes the cameras

        Ensure that valid proxies to the Nao cameras exist
        and subscribe to the cameras.

        Parameters
        ----------
        pip : str
            The IP of the agent for which to process the camera images.
        pport : int
            The port of the agent for which to process the camera images.

        """
        try:
            # noinspection PyBroadException
            try:
                self._cameraProxy.ping()
            except:
                self._cameraProxy = ALProxy("ALVideoDevice", pip, pport)

                color_space = vision_definitions.kBGRColorSpace
                fps = 30
                self._subscriber_btm_id = self._cameraProxy.subscribeCamera("ImageProcessor", kBottomCamera,
                                                                            self._resolution, color_space, fps)
                self._subscriber_top_id = self._cameraProxy.subscribeCamera("ImageProcessor", kTopCamera,
                                                                            self._resolution, color_space, fps)
        except:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_exception(exc_type, exc_value, exc_traceback)
            sys.exit(1)

    def _get_image(self, camera_id):
        """Returns the image for the given camera.

        Capture the camera image from the camera with the given id.

        Parameters
        ----------
        camera_id : {kTopCamera, kBottomCamera}
            Whether the bottom or the top camera captured
            the image.

        Returns
        -------
        CameraImage
            The captured camera image.

        """
        subscriber_id = self._subscriber_btm_id if camera_id == kBottomCamera else self._subscriber_top_id
        cam_img = self._cameraProxy.getImageRemote(subscriber_id)
        try:
            img_width = cam_img[0]
            img_height = cam_img[1]
            im = Image.frombytes("RGB", (img_width, img_height), cam_img[6])
            timestamp = cam_img[4] + cam_img[5] / micro
            return CameraImage(np.array(im), img_width, img_height, timestamp, cam_img[7])
        except:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_exception(exc_type, exc_value, exc_traceback)
            sys.exit(1)

    def _generate_id(self, pip, pport):
        return "%s.%s:%s:%i:%i" % (self.__class__.__module__, self.__class__.__name__, pip, pport, next(self._ids))
