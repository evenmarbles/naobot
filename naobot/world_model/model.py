from __future__ import division, print_function, absolute_import

import os
from .objects import Ball

from mlpy.tools.configuration import ConfigMgr
from mlpy.agents.world import WorldModel


class NaoWorldModel(WorldModel):
    """Nao world object manager.

    Notes
    -----
    Follows the Borg design pattern, meaning that all instances of
    the world model share state.

    """
    _sensors = {
        "joint": {},
        "inertial": {},
        "foot": {},
        "bumper": {},
    }
    _positions = {
        "effector": {},
        "joint": {},
        "camera": {}
    }
    _robot_model = None
    _robot_info = None

    _current_time = None

    @property
    def robot_model(self):
        return self._robot_model

    def __init__(self):
        super(NaoWorldModel, self).__init__()

        self.has_fallen = False

        self.add_object('ball', Ball())

    def get_joint(self, name):
        return self._sensors["joint"][name]

    def get_inertial(self, name):
        return self._sensors["inertial"][name]

    def get_foot(self, name):
        return self._sensors["foot"][name]

    def get_bumper(self, name):
        return self._sensors["bumper"][name]

    def get_sensors(self, name):
        """
        Returns the internal state based on the name.

        :type name: str
        :rtype: dict[str, dict[str, float]]
        """
        return self._sensors[name]

    def get_effector_pos(self, name):
        return self._positions["effector"][name]

    def get_joint_pos(self, name):
        return self._positions["joint"][name]

    def get_camera_pos(self, name):
        return self._positions["camera"][name]

    def get_positions(self, name):
        """
        Returns the internal state based on the name.

        :type name: str
        :rtype: dict[str, dict[str, float]]
        """
        return self._positions[name]

    def set_robot_params(self, robot_type, robot_version):
        self._robot_model = (robot_type, robot_version)

        try:
            robot_cfg = ConfigMgr(
                os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config')) + "/robotInfo.cfg")
            self._robot_info = robot_cfg.get(robot_type + "." + robot_version)
        except KeyError as e:
            exit("The key \'" + e.message + "\' is required to be configured!")

    def get_robot_info(self, name):
        """
        Returns the information on a robot link or joint
        :rtype : float | tuple[float, float]
        """
        if self._robot_info is None:
            self.set_robot_params("naoH25", "VERSION_40")
        return self._robot_info[name]
