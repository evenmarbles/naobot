from __future__ import division, print_function, absolute_import

from naoqi import ALProxy

from mlpy.modules import Module

from .world_model.model import NaoWorldModel


class Sensors(Module):
    """Nao's sensor class.

    An interface to NAOqi's ALMemory [1]_ module encapsulating the functionality.

    This class reads the joint angles, the inertial information, the force
    sensitive resistance (FSR), and the bumper values from the Nao's memory at
    each time step.

    Parameters
    ----------
    pip : str
        The IP of the agent for which to process the camera images.
    pport : int
        The port of the agent for which to process the camera images.

    Notes
    -----
    The joint angles are measured in radians. And the FSR are measured in kilogram
    at the feet. Bumpers return a 2-state float value 0.0 (unpressed) or 1.0 (pressed).

    The inertial sensors supported are:

        Gyroscope
            Extract the direct rotation speed values in rad/s.
        Accelerometer
            3-axis acceleration in m/s.
        Angles
            Three inclination angles in radians of the robot body are computed.

    For more information see [2]_.

    References
    ----------
    .. [1] `ALMemory <http://doc.aldebaran.com/2-1/naoqi/core/almemory.html>`_
    .. [2] `NAO - Actuator & Sensor list <http://doc.aldebaran.com/2-1/family/nao_dcm/actuator_sensor_names.html>`_

    """
    _jointNames = ["HeadYaw", "HeadPitch", "LShoulderRoll", "LShoulderPitch", "LElbowYaw",
                   "LElbowRoll", "RShoulderRoll", "RShoulderPitch", "RElbowYaw", "RElbowRoll",
                   "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch",
                   "LAnkleRoll", "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch",
                   "RAnklePitch", "RAnkleRoll"]
    _inertialNames = ["AccX", "AccY", "AccZ", "GyroscopeX", "GyroscopeY", "AngleX", "AngleY"]
    _footNames = ["LFoot/FSR/FrontLeft", "LFoot/FSR/FrontRight", "LFoot/FSR/RearLeft", "LFoot/FSR/RearRight",
                  "RFoot/FSR/FrontLeft", "RFoot/FSR/FrontRight", "RFoot/FSR/RearLeft", "RFoot/FSR/RearRight"]
    _bumperNames = ["LFoot/Bumper/Left", "LFoot/Bumper/Right", "RFoot/Bumper/Left", "RFoot/Bumper/Right"]

    def __init__(self, pip, pport):
        super(Sensors, self).__init__(self._generate_id(pip, pport))

        self._memoryProxy = None

        self._joint = NaoWorldModel().get_sensors("joint")
        self._inertial = NaoWorldModel().get_sensors("inertial")
        self._foot = NaoWorldModel().get_sensors("foot")
        self._bumper = NaoWorldModel().get_sensors("bumper")

        self._init_joint()
        self._init_inertial()
        self._init_foot()
        self._init_bumper()

    def __getstate__(self):
        d = super(Sensors, self).__getstate__()
        d['_mid'] = self._mid
        return d

    def enter(self, t):
        """Perform preliminary setup.

        Ensure that valid proxies to the Nao memory exist.

        Parameters
        ----------
        t : float
            The current time (sec).

        """
        super(Sensors, self).enter(t)

        try:
            try:
                self._memoryProxy.ping()
            except:
                self._memoryProxy = ALProxy("ALMemory", self.mid.split(':')[1], int(self.mid.split(':')[2]))
                self._read_sensors()
        except:
            pass

    def update(self, dt):
        """Update Nao's sensor information.

        Read the positions and rotation angles for all joints, inertial
        sensors, foot sensors, and bumpers. Also check if the robot
        has fallen.

        Parameters
        ----------
        dt : float
            The elapsed time (sec)

        """
        super(Sensors, self).update(dt)
        self._read_sensors()

    def _read_sensors(self):
        self._has_fallen()
        self._read_joint()
        self._read_inertial()
        self._read_foot()
        self._read_bumper()

    def _has_fallen(self):
        """Identify if the robot has fallen."""
        NaoWorldModel().has_fallen = self._memoryProxy.getData("footContact") == 0.0

    def _init_joint(self):
        """Initialize all joint angles."""
        for x in self._jointNames:
            self._joint[x] = 0.0

    def _init_inertial(self):
        """Initialize all inertial information."""
        for x in self._inertialNames:
            self._inertial[x] = 0.0

    def _init_foot(self):
        """Initialize all FSR values."""
        for x in self._footNames:
            self._foot[x] = 0.0

    def _init_bumper(self):
        """Initialize all bumper values."""
        for x in self._bumperNames:
            self._bumper[x] = 0.0

    def _read_joint(self):
        """Read the angle of all joints.

        Reads the joint angles in radians.

        """
        for x in self._jointNames:
            self._joint[x] = self._memoryProxy.getData("Device/SubDeviceList/" + x + "/Position/Sensor/Value")

    def _read_inertial(self):
        """Extract information from the inertial sensor in the center of the body.

        Gyroscope
            Extract the direct rotation speed values in rad/s.
        Accelerometer
            3-axis acceleration in m/s.
        Angles
            Three inclination angles in radians of the robot body are computed.

        """
        for x in self._inertialNames:
            self._inertial[x] = self._memoryProxy.getData("Device/SubDeviceList/InertialSensor/" + x + "/Sensor/Value")

    def _read_foot(self):
        """Read the force sensitive resistance (FSR) of all foot sensors.

        These sensors measure a resistance change according to the pressure applied.
        The measurements are in kilograms.

        """
        for x in self._footNames:
            self._foot[x] = self._memoryProxy.getData("Device/SubDeviceList/" + x + "/Sensor/Value")

    def _read_bumper(self):
        """Read all bumpers.

        This identifies whether the bumper was pressed or not.

        """
        for x in self._bumperNames:
            self._bumper[x] = self._memoryProxy.getData("Device/SubDeviceList/" + x + "/Sensor/Value")

    def _generate_id(self, pip, pport):
        return "%s.%s:%s:%i:%i" % (self.__class__.__module__, self.__class__.__name__, pip, pport, next(self._ids))
