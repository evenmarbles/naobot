from __future__ import division, print_function, absolute_import

import sys
import traceback

import motion
import almath
from naoqi import ALProxy

from mlpy.modules import Module
from mlpy.auxiliary.misc import listify

from .world_model.model import NaoWorldModel


class NaoMotionController(Module):
    """The controller for Nao's motion.

    An interface to NAOqi's ALMotion [1]_ and ALRobotPosture [2]_ modules
    encapsulating their functionality.

    Parameters
    ----------
    pip : str
        The IP of the agent for which to process the camera images.
    pport : int
        The port of the agent for which to process the camera images.
    default_frame : {FRAME_ROBOT, FRAME_TORSO, FRAME_WORLD}
        The frame to use by default
    use_sensor_values : bool
        Whether to use sensor values or not. Default is True.

    References
    ----------
    .. [1] `ALMotion <http://doc.aldebaran.com/2-1/naoqi/motion/almotion.html>`_
    .. [2] `ALRobotPosture <http://doc.aldebaran.com/2-1/naoqi/motion/alrobotposture.html>`_

    """
    # FRAME
    FRAME_ROBOT = motion.FRAME_ROBOT
    FRAME_TORSO = motion.FRAME_TORSO
    FRAME_WORLD = motion.FRAME_WORLD

    # AXIS MASK
    AXIS_MASK_X = motion.AXIS_MASK_X
    AXIS_MASK_Y = motion.AXIS_MASK_Y
    AXIS_MASK_Z = motion.AXIS_MASK_Z
    AXIS_MASK_WX = motion.AXIS_MASK_WX
    AXIS_MASK_WY = motion.AXIS_MASK_WY
    AXIS_MASK_WZ = motion.AXIS_MASK_WZ
    AXIS_MASK_ALL = motion.AXIS_MASK_ALL
    AXIS_MASK_VEL = motion.AXIS_MASK_VEL
    AXIS_MASK_ROT = motion.AXIS_MASK_ROT

    _effectorNames = ["Head", "LArm", "RArm", "LLeg", "RLeg", "Torso"]
    _jointNames = ["HeadYaw", "HeadPitch", "LShoulderRoll", "LShoulderPitch", "LElbowYaw",
                   "LElbowRoll", "RShoulderRoll", "RShoulderPitch", "RElbowYaw", "RElbowRoll",
                   "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch",
                   "LAnkleRoll", "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch",
                   "RAnklePitch", "RAnkleRoll"]
    _cameraNames = ["CameraTop", "CameraBottom"]

    def __init__(self, pip, pport, default_frame=None, use_sensor_values=None):
        super(NaoMotionController, self).__init__(self._generate_id(pip, pport))

        self._motion_proxy = None
        self._posture_proxy = None

        self._task_id = None

        self._default_frame = default_frame if default_frame is not None else NaoMotionController.FRAME_ROBOT
        self._use_sensor_values = use_sensor_values if use_sensor_values is not None else True

        self._initialize(pip, pport)

        robot_config = self._motion_proxy.getRobotConfig()
        NaoWorldModel().set_robot_params(robot_config[1][0], robot_config[1][2])

        self._effector = NaoWorldModel().get_positions("effector")
        self._joint = NaoWorldModel().get_positions("joint")
        self._camera = NaoWorldModel().get_positions("camera")

        self._init_effector()
        self._init_joint()
        self._init_camera()

    def reset(self, t, **kwargs):
        """Resets the motion controller.

        Ensure that all proxies are valid.

        Parameters
        ----------
        t : float
            The current time (sec)
        kwargs : dict, optional
            Non-positional parameters, optional.

        """
        super(NaoMotionController, self).reset(t, **kwargs)
        self._initialize(self.mid.split(':')[1], int(self.mid.split(':')[2]))

    def enter(self, t):
        """Perform preliminary setup.

        Parameters
        ----------
        t : float
            The current time (sec).

        """
        super(NaoMotionController, self).enter(t)

        self.go_to_default_posture()
        self.update(0.0)

    def update(self, dt):
        """Update the motion controller.

        Read the positions and rotation angles for all effectors,
        joints, and cameras.

        Parameters
        ----------
        dt : float
            The elapsed time (sec)

        """
        super(NaoMotionController, self).update(dt)

        self._update_effector()
        self._update_joint()
        self._update_camera()

    def go_to_posture(self, name, speed):
        """Go to predefined posture.

        This encapsulates the function `goToPosture
        <http://doc.aldebaran.com/2-1/naoqi/motion/alrobotposture-api.html#ALRobotPostureProxy::goToPosture__ssC.floatC>`_
        from the ALRobotPosture module.

        Parameters
        ----------
        name : str
            The name of the posture.
        speed : float
            Relative speed between 0.0 and 1.0.

        Notes
        -----
        This is a blocking call.

        """
        self._posture_proxy.goToPosture(name, speed)

    def go_to_default_posture(self):
        """Go to default posture.

        This encapsulates the function `goToPosture
        <http://doc.aldebaran.com/2-1/naoqi/motion/alrobotposture-api.html#ALRobotPostureProxy::goToPosture__ssC.floatC>`_
        from the ALRobotPosture module.

        The default posture is `StandInit` and the agent
        transitions to this posture with a speed of 0.5.

        """
        self._posture_proxy.goToPosture("StandInit", 0.5)

    def interpolate_angles(self, joints, angles, times, is_absolute=True):
        """Interpolates joints to a target angles.

        Interpolates one or more joints to a target angle or along timed trajectories.

        This encapsulates the function `angleInterpolation
        <http://doc.aldebaran.com/2-1/naoqi/motion/control-joint-api.html#ALMotionProxy::angleInterpolation__AL::ALValueCR.AL::ALValueCR.AL::ALValueCR.bCR>`_
        from the ALMotion module.

        Parameters
        ----------
        joints : str or list[str]
            The name(s) of the joints or chains.
        angles : float or list[float] or list[list[float]]
            The target angle(s) in radians the joints are interpolated to.
        times : float or list[float] or list[list[float]]
            The time in seconds for the interpolation.
        is_absolute : bool, optional
            If true, the movement is described in absolute angles, else the
            angles are relative to the current angels. Default is True.

        Notes
        -----
        This is a non-blocking call.

        """
        try:
            self._task_id = self._motion_proxy.post.angleInterpolation(joints, angles, times, is_absolute)
        except:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_exception(exc_type, exc_value, exc_traceback)
            sys.exit(1)

    def interpolate_positions(self, effectors, delta_pos, times, frames=None, axis_masks=None):
        """Interpolate end-effectors to target positions and orientations.

        Moves end-effectors to the given position and orientation over time.

        This encapsulates the function `positionInterpolation
        <http://doc.aldebaran.com/2-1/naoqi/motion/control-cartesian-api.html#almotionproxy-positioninterpolations1>`_
        from the ALMotion module.

        Parameters
        ----------
        effectors : str or list[str]
            List of effector names. Effector name could be "Torso" or chain name.
        delta_pos : ndarray[float] or list[ndarray[float]] or list[list[ndarray[float]]]
            The relative position changes for each effector.
        times : list[float] or list[list[float]]
            Vector of times in seconds corresponding to the path points.
        frames : list[int], optional
            List of task frames {FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2 }.
            Default is FRAME_ROBOT.
        axis_masks : list[int]
            List of axis masks. True for axes that should be controlled. Default is
            AXIS_MASK_ALL.

        Notes
        -----
        This is a non-blocking call.

        """
        effectors = listify(effectors)
        n = len(effectors)

        frames = listify(frames) if frames is not None else [NaoMotionController.FRAME_ROBOT] * n
        assert (len(frames) == n), "effectors and frames must have the same number of elements"
        axis_masks = listify(axis_masks) if axis_masks is not None else [NaoMotionController.AXIS_MASK_ALL] * n
        assert (len(axis_masks) == n), "effectors and axis_masks must have the same number of elements"

        try:
            path = []
            for e, d, f in zip(effectors, listify(delta_pos), frames):
                path.append(self._calculate_target_pos(e, d, f))
            self._task_id = self._motion_proxy.post.positionInterpolations(effectors, frames, path, axis_masks, times)
        except:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_exception(exc_type, exc_value, exc_traceback)
            sys.exit(1)

    def transform(self, effectors, delta_tf, times, frames=None, axis_masks=None):
        """Moves end-effector to the given transforms over time.

        This encapsulates the function `transformInterpolation
        <http://doc.aldebaran.com/2-1/naoqi/motion/control-cartesian-api.html#almotionproxy-transforminterpolations1>`_
        from the ALMotion module.

        Parameters
        ----------
        effectors : str or list[str]
            List of effector names. Effector name could be "Torso" or chain name.
        delta_tf : ndarray[float] or list[ndarray[float]] or list[list[ndarray[float]]]
            The relative position changes.
        times : list[float] or list[list[float]]
            List of times in seconds corresponding to the path points.
        frames : list[int], optional
            List of task frames {FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2 }.
            Default is FRAME_ROBOT.
        axis_masks : list[int], optional
            List of axis masks. True for axes that should be controlled. Default is
            AXIS_MASK_ALL.

        Notes
        -----
        This is a non-blocking call.

        """
        effectors = listify(effectors)
        n = len(effectors)

        frames = listify(frames) if frames is not None else [NaoMotionController.FRAME_ROBOT] * n
        assert (len(frames) == n), "effectors and frames must have the same number of elements"
        axis_masks = listify(axis_masks) if axis_masks is not None else [NaoMotionController.AXIS_MASK_ALL] * n
        assert (len(axis_masks) == n), "effectors and axis_masks must have the same number of elements"

        try:
            path = []
            for e, d, f in zip(effectors, listify(delta_tf), frames):
                path.append(self._calculate_target_tf(e, d, f))
            self._task_id = self._motion_proxy.post.transformInterpolations(effectors, frames, path, axis_masks, times)
        except:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_exception(exc_type, exc_value, exc_traceback)
            sys.exit(1)

    def move_joint(self, name, angle, fraction_max_speed=1.0):
        """ Sets the joint angle directly.

        This encapsulates the function `setAngles
        <http://doc.aldebaran.com/2-1/naoqi/motion/control-joint-api.html#ALMotionProxy::setAngles__AL::ALValueCR.AL::ALValueCR.floatCR>`_
        from the ALMotion module.

        Parameters
        ----------
        name: str | list[str]
            The name(s) of the joints or chains.
        angle: float | list[floats]
            The angles in radians to which the angle is set.
        fraction_max_speed: float, optional
            The fraction of maximum speed to use. Default is 1.0.

        Notes
        -----
        This is a non-blocking call.

        """
        self._motion_proxy.setAngles(name, angle, fraction_max_speed)

    def wait(self):
        """Waits until the end of the current task"""
        self._motion_proxy.wait(self._task_id, 0)

    def is_running(self):
        """Check if the current task is still running."""
        if not self._task_id:
            return False
        return self._motion_proxy.isRunning(self._task_id)

    def _initialize(self, pip, pport):
        """Initializes the motion controller

        Ensure that valid proxies to the Nao motion and robot
        posture exist.

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
                self._motion_proxy.ping()
                self._posture_proxy.ping()
            except:
                self._motion_proxy = ALProxy("ALMotion", pip, pport)
                self._posture_proxy = ALProxy("ALRobotPosture", pip, pport)

                self._motion_proxy.wakeUp()
        except:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_exception(exc_type, exc_value, exc_traceback)
            sys.exit(1)

    def _init_effector(self):
        """Initialize the position and orientation of all effectors."""
        for x in self._effectorNames:
            self._effector[x] = 0.0

    def _init_joint(self):
        """Initialize the position and orientation of all joints."""
        for x in self._jointNames:
            self._joint[x] = 0.0

    def _init_camera(self):
        """Initialize the position and orientation of all cameras."""
        for x in self._cameraNames:
            self._camera[x] = 0.0

    def _update_effector(self):
        """Update the position and orientation of all effectors."""
        for x in self._effectorNames:
            self._effector[x] = self._motion_proxy.getPosition(x, self._default_frame, self._use_sensor_values)

    def _update_joint(self):
        """Update the position and orientation of all joints."""
        for x in self._jointNames:
            self._joint[x] = self._motion_proxy.getPosition(x, self._default_frame, self._use_sensor_values)

    def _update_camera(self):
        """Update the position and orientation of all cameras."""
        for x in self._cameraNames:
            self._camera[x] = self._motion_proxy.getPosition(x, self._default_frame, self._use_sensor_values)

    def _calculate_target_tf(self, effector, delta_tf, frame):
        """Calculate the target transform.

        Calculate the target transform from the current position of the effector
        and the relative position change.

        Parameters
        ----------
        effector : str
            String of effector name.
        delta_tf : ndarray[float]
            The relative position change.
        frame : {FRAME_TORSO, FRAME_WORLD, FRAME_ROBOT}
            The task frame .

        """
        init_tf = almath.Transform(self._motion_proxy.getTransform(effector, frame, self._use_sensor_values))

        delta_tf = listify(delta_tf)
        d = delta_tf.pop(0)

        target_tf = init_tf * almath.Transform().fromPosition(*d)
        path = list(target_tf.toVector())

        if delta_tf:
            path = [path]
            for d in delta_tf:
                target_tf *= almath.Transform().fromPosition(*d)
                path.append(list(target_tf.toVector()))
        return path

    def _calculate_target_pos(self, effector, delta_pos, frame):
        """Calculate the target position.

        Calculate the target position from the current position of the effector
        and the relative position change.

        Parameters
        ----------
        effector : str
            String of effector name.
        delta_pos : ndarray[float]
            The relative position change.
        frame : {FRAME_TORSO, FRAME_WORLD, FRAME_ROBOT}
            The task frame .

        """
        init_pos = self._motion_proxy.getPosition(effector, frame, self._use_sensor_values)

        delta_pos = listify(delta_pos)
        d = delta_pos.pop(0)

        target_pos = almath.Position6D(init_pos)
        target_pos += almath.Position6D(d)
        path = list(target_pos.toVector())

        if delta_pos:
            path = [path]
            for d in delta_pos:
                target_pos += almath.Position6D(d)
                path.append(list(target_pos.toVector()))
        return path

    def _generate_id(self, pip, pport):
        return "%s.%s:%s:%i:%i" % (self.__class__.__module__, self.__class__.__name__, pip, pport, next(self._ids))
