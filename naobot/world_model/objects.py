from __future__ import division, print_function, absolute_import

from mlpy.auxiliary.datastructs import Point2D, Vector3D
from mlpy.agents.world import WorldObject


class NaoWorldObject(WorldObject):
    """Nao world object class.

    Attributes
    ----------
    location : Points3D
        The objects current location.
    timestamp : float
        The timestamp the object was last seen (the image was captured).
    confidence : float
        The level of confidence for the object's information based on
        when the object was last seen.
    camera_id : {kTopCamera, kBottomCamera}
        Whether the bottom or the top camera captured
        the image of the object.

    Notes
    -----
    All Nao world object should inherit from this class.

    """
    def __init__(self):
        super(NaoWorldObject, self).__init__()

        self.camera_id = 0


class Ball(NaoWorldObject):
    """The ball object.

    Attributes
    ----------
    location : Points3D
        The objects current location.
    timestamp : float
        The timestamp the object was last seen (the image was captured).
    confidence : float
        The level of confidence for the object's information based on
        when the object was last seen.
    camera_id : {kTopCamera, kBottomCamera}
        Whether the bottom or the top camera captured
        the image of the ball.
    resolution : tuple
        The resolution of the camera image.
    image_center : Point2D
        The center of the ball in the image.
    image_radius : float
        The radius of the ball in the image.
    velocity : Vector3D
        The velocity with which the ball is moving.
    elevation : float
        The elevation of the ball.

    """
    def __init__(self):
        super(Ball, self).__init__()

        self.resolution = ()
        self.image_center = Point2D()
        self.image_radius = 0.0

        self.velocity = Vector3D()
        self.elevation = 0.0

    def enter(self, t):
        """Enter the ball object.

        Parameters
        ----------
        t : float
            The current time (sec).

        """
        super(Ball, self).enter(t)

    def update(self, dt):
        """Update the ball information.

        Update elevation, velocity, etc. based on the camera image.

        Parameters
        ----------
        dt : float
            The elapsed time (sec)

        .. todo::
            Update elevation, velocity, etc.

        """
        super(Ball, self).update(dt)
        # TODO: Update elevation, velocity, etc.
