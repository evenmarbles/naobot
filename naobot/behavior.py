from __future__ import division, print_function, absolute_import

import sys
import traceback
import importlib

from mlpy.agents.fsm import StateMachine, FSMState
from mlpy.auxiliary.io import import_module_from_path

from .world_model.model import NaoWorldModel
from .kinematics import NaoMotionController


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

    def reset(self, t, **kwargs):
        """Resets Nao's sensors.

        Ensure that all proxies are valid.

        Parameters
        ----------
        t : float
            The current time (sec)
        kwargs : dict, optional
            Non-positional parameters, optional.

        """
        super(NaoState, self).reset(t, **kwargs)

        if self._motion:
            self._motion.reset(t, **kwargs)

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


class NaoBehavior(StateMachine):
    """The finite state machine for the soccer bot behavior.

    This class controls Nao's behavior based on state transitions
    specified in the behavior configuration.

    Parameters
    ----------
    pip : str
        The IP of the agent for which to process the camera images.
    pport : int
        The port of the agent for which to process the camera images.
    states : FSMState | list[FSMState], optional
        A list of states.
    initial : str, optional
        The initial state.
    transitions : list[dict] | list[list], optional
        Transition information.
    onupdate : list[dict] | list[list], optional
        Callback information to be executed on update.
    model : object, optional
        Reference to the object owning the FSM.
    configuration : ConfigMgr, optional
        The configuration manager containing the information
        for setting up the FSM. This allowing a data driven
        FSM setup.

    """
    def __init__(self, pip, pport, states=None, initial=None, transitions=None,
                 onupdate=None, model=None, configuration=None):
        super(NaoBehavior, self).__init__(states, initial, transitions, onupdate)
        self._load_fsm(model, NaoMotionController(pip, pport), configuration)

    # noinspection PyUnresolvedReferences
    def _load_fsm(self, model, motion, cfg):
        """Load the FSM from the configuration file.

        Parameters
        ----------
        model: object
            Reference to the object owning the FSM.
        motion: NaoMotionController
            The motion controller, controlling Nao's motions.
        cfg: ConfigMgr
            The configuration manager containing the information
            for setting up the FSM. This allowing a data driven
            FSM setup.

        """
        try:
            module = import_module_from_path(cfg.get("Module"), "task_fsm")

            self._initial = cfg.get("Initial")

            for state in cfg.get("States"):
                kwargs = {}
                if isinstance(state, dict):
                    kwargs = state.itervalues().next()
                    state = state.iterkeys().next()
                state_c = getattr(module, state)(motion, **kwargs)

                self.add_states(state_c)

            for t in cfg.get("Transitions"):
                for key in t.iterkeys():
                    c = None
                    if key == "before":
                        c = "before"
                    if key == "after":
                        c = "after"
                    if key == "conditions":
                        c = "conditions"

                    if c is not None:
                        if c in ["before", "after"]:
                            if t[c]["model"] == model.__class__.__name__:
                                t[c] = getattr(model, t[c]["func"])
                            else:
                                k = t[c]["model"].rfind(".")
                                n = t[c]["model"][:k]
                                m = t[c]["model"][k+1:]
                                # noinspection PyUnusedLocal
                                module = importlib.import_module(n)
                                t[c]["model"] = eval("module." + m)
                        if c == "conditions":
                            for i, cond in enumerate(t[c]):
                                t[c][i] = eval(t[c][i])

                self.add_transition(t["event"], t["source"], t["dest"],
                                    conditions=t["conditions"] if "conditions" in t else None,
                                    before=t["before"] if "before" in t else None,
                                    after=t["after"] if "after" in t else None)

            for u in cfg.get("OnUpdate"):
                for key in u.iterkeys():
                    c = None
                    if key == "onupdate":
                        c = "onupdate"
                    if key == "conditions":
                        c = "conditions"

                    if c is not None:
                        if c in "onupdate":
                            if u[c]["model"] == model.__class__.__name__:
                                u[c] = getattr(model, u[c]["func"])
                            else:
                                k = u[c]["model"].rfind(".")
                                n = u[c]["model"][:k]
                                m = u[c]["model"][k+1:]
                                # noinspection PyUnusedLocal
                                module = importlib.import_module(n)
                                u[c]["model"] = eval("module." + m)
                        if c == "conditions":
                            for i, cond in enumerate(u[c]):
                                u[c][i] = eval(u[c][i])

                self.add_onupdate(u["source"], onupdate=u["onupdate"] if "onupdate" in u else None,
                                  conditions=u["conditions"] if "conditions" in u else None)
        except KeyError:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_exception(exc_type, exc_value, exc_traceback)
            sys.exit(1)
        except:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_exception(exc_type, exc_value, exc_traceback)
            sys.exit(1)
