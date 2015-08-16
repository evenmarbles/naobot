from __future__ import division, print_function, absolute_import

from mlpy.agents import Agent
from mlpy.auxiliary.datasets import DataSet
from mlpy.agents.fsm import StateMachine

from naobot.kinematics import NaoMotionController
from .sensors import Sensors, NaoWorldModel
from .vision.image import ImageProcessor


class NaoBot(Agent):
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

    def __init__(self, pip, pport, module_type, task, fsm_filename, keep_history=False,
                 dataset_params=None, *args, **kwargs):
        assert (isinstance(pip, basestring) and isinstance(pport, int)), \
            "The player IP must be of type string and the player port of type integer."
        super(NaoBot, self).__init__(pip + ":" + str(pport), module_type, task, *args, **kwargs)

        self._keep_history = keep_history

        if self._keep_history:
            dataset_params = dataset_params if dataset_params is not None else {}
            self._history = DataSet(**dataset_params)
            self._history.load()

        self._world_model = NaoWorldModel()
        self._sensors = Sensors(pip, pport)
        self._vision = ImageProcessor(pip, pport)

        self._behavior = StateMachine()
        self._behavior.load_from_file(self, fsm_filename, motion=NaoMotionController(pip, pport))

    def reset(self, t, **kwargs):
        """Reset the naobot's state.

        Parameters
        ----------
        t : float
            The current time (sec).
        kwargs : dict, optional
            Non-positional parameters.

        """
        super(NaoBot, self).reset(t, **kwargs)

        if not self._task.is_complete():
            if self._keep_history:
                self._history.new_sequence()

            self._sensors.reset(t, **kwargs)
            self._vision.reset(t, **kwargs)
            self._behavior.reset(t, **kwargs)

    def enter(self, t):
        """Enter naobot and the agent module.

        Perform initialization tasks here.

        Parameters
        ----------
        t : float
            The current time (sec).

        """
        super(NaoBot, self).enter(t)

        if self._keep_history:
            self._history.new_sequence()

        self._sensors.enter(t)
        self._vision.enter(t)
        self._world_model.enter(t)
        self._behavior.enter(t)

    def update(self, dt):
        """Update naobot and the agent module.

        Naobot and the agent module are updated at every time step
        in the program loop.

        Parameters
        ----------
        dt : float
            The elapsed time (sec)

        """
        super(NaoBot, self).update(dt)

        self._sensors.update(dt)
        self._vision.update(dt)
        self._world_model.update(dt)
        self._behavior.update(dt)

    def exit(self):
        """Exit the agent and the agent module.

        Perform cleanup tasks here.

        """
        super(NaoBot, self).exit()

        self._behavior.exit()
        self._vision.exit()
        self._sensors.exit()
        self._world_model.exit()

    def motion_complete(self):
        """The motion has completed.

        The method is called by the behavior when the motion has completed.

        """
        state = self._task.sensation(**{"state": self._behavior.current_state})

        if not self._task.termination_requested():
            if state is not None:
                self._module.execute(state)

                if self._keep_history:
                    if not self._history.has_field("state"):
                        self._history.add_field("state", len(state), dtype=state.dtype, description=state.description)
                    self._history.append("state", state)

                    if not self._history.has_field("label"):
                        self._history.add_field("label", 1, dtype=DataSet.DTYPE_OBJECT)
                    self._history.append("label", state.name)

        if (self._task.is_episodic and state.is_terminal()) or self._task.termination_requested():
            if self._keep_history:
                self._history.save()

            self._task.terminate(True)
            self._task.request_termination(False)

    def choose_action(self):
        """Determine the next action based on the actor module.

        This method is called by the behavior.

        """
        action = self._module.get_next_action()
        if action is not None:
            self._behavior.post_event(action.name, **{"action": action})

            if self._keep_history:
                if not self._history.has_field("act"):
                    self._history.add_field("act", len(action), dtype=action.dtype, description=action.description)
                self._history.append("act", action)
        else:
            self._behavior.post_event("no-op", **{"delay": self._task.event_delay})
            self._task.request_termination(True)

    def process_motion(self):
        """Process the motion.

        This method is called after updating the state.

        """
        self.motion_complete()

        if self._task.is_complete():
            return

        self.choose_action()
