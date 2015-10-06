from __future__ import division, print_function, absolute_import

import time
from mlpy.agents.modelbased import Bot
from mlpy.agents.fsm import StateMachine

from naobot.kinematics import NaoMotionController
from .sensors import Sensors, NaoWorldModel
from .vision.image import ImageProcessor


class NaoBot(Bot):
    """The Naobot agent class.

    This class controls the Nao robot.

    Parameters
    ----------
    pip : str
        The IP of the agent for which to process the camera images.
    pport : int
        The port of the agent for which to process the camera images.
    module_type : str
        The agent module type, which is the name of the class.
    task: Task
        The task the agent must complete.
    fsm_filename : str
        Name of the file containing the fsm configuration information.
    keep_history : bool, optional
        If true a history of states and actions performed by the agent
        are recorded and saved to file. Default is False.
    dataset_params : dict
        Non-positional arguments passed to the dataset during instantiation.
    args: tuple
        Positional parameters passed to the agent module.
    kwargs: dict
        Non-positional parameters passed to the agent module.

    """

    @property
    def pip(self):
        ip, _ = self.mid.split(":")
        return ip

    @property
    def pport(self):
        _, port = self.mid.split(":")
        return int(port)

    def __init__(self, pip, pport, fsm_filename):
        assert (isinstance(pip, basestring) and isinstance(pport, int)), \
            "The player IP must be of type string and the player port of type integer."
        super(NaoBot, self).__init__(pip + ":" + str(pport))

        self._world_model = None
        self._sensors = None
        self._vision = None
        self._behavior = None

        self._fsm_filename = fsm_filename

    def __getstate__(self):
        d = super(NaoBot, self).__getstate__()
        d['_mid'] = self._mid
        return d

    def init(self):
        pip, pport = self.mid.split(':')[0], int(self.mid.split(':')[1])
        self._world_model = NaoWorldModel()
        self._sensors = Sensors(pip, pport)
        self._vision = ImageProcessor(pip, pport)

        self._behavior = StateMachine()
        self._behavior.load_from_file(self, self._fsm_filename, motion=NaoMotionController(pip, pport))

    def enter(self, t):
        """Enter naobot and the agent module.

        Perform initialization tasks here.

        Parameters
        ----------
        t : float
            The current time (sec).

        """
        while True:
            ready = True
            # noinspection PyBroadException
            try:
                self._sensors.enter(t)
                self._vision.enter(t)
                self._behavior.enter(t)
            except:
                ready = False
            if ready:
                break
            time.sleep(1)

        self._world_model.enter(t)

    def update(self, dt, action=None):
        """Update naobot and the agent module.

        Naobot and the agent module are updated at every time step
        in the program loop.

        Parameters
        ----------
        dt : float
            The elapsed time (sec)

        """
        if action is not None:
            self._behavior.post_event(action.name, **{"action": action})

        self._sensors.update(dt)
        self._vision.update(dt)
        self._world_model.update(dt)
        self._behavior.update(dt)

    def exit(self):
        """Exit the agent and the agent module.

        Perform cleanup tasks here.

        """
        self._behavior.exit()
        self._vision.exit()
        self._sensors.exit()
        self._world_model.exit()

    def motion_complete(self):
        """Process the motion.

        This method is called after updating the state.

        """
        self.set_is_ready(True)
