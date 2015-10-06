from __future__ import division, print_function, absolute_import

from mlpy.agents.fsm import FSMState

from .world_model.model import NaoWorldModel


class NaoState(FSMState):
    """FSM base state for all bot states.

    Parameters
    ----------
    motion : NaoMotionController
        The motion controller.

    """

    # noinspection PyArgumentList
    def __init__(self, motion):
        super(NaoState, self).__init__()

        self._motion = motion

    def enter(self, t, *args, **kwargs):
        """State initialization.

        Parameters
        ----------
        t : float
            The current time (sec)

        """
        super(NaoState, self).enter(t)
        NaoWorldModel().set_fsm_state(self.name)

    def update(self, dt):
        """Update the state.

        Update the state and handle state transitions based on events.

        Parameters
        ----------
        dt : float
            The elapsed time (sec)

        Returns
        -------
        Event
            The transition event.

        """
        super(NaoState, self).update(dt)

        self._motion.update(dt)

        if NaoWorldModel().has_fallen:
            return "has_fallen"
