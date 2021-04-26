from abc import ABCMeta
from dynamic_stack_decider.abstract_stack_element import AbstractStackElement


class AbstractActionElement(AbstractStackElement, metaclass=ABCMeta):
    """
    One action is similar to a state of an FSM.
    As in this case, the system stays in this state in contrast to the decision elements which are called only for determining the active action.
    It defines the actions which the robot does, for example performing a kick.
    Another example is an action which takes care of going to the ball, the action remains on top of the stack until the ball is reached.
    The action only makes decisions which are necessary for its purpose, like some adjustments to the kicking movement.
    Actions do not push further elements on the stack but command actions on lower-level modules like new movement goals.
    If the action is complete, it can remove itself from the stack by performing a pop command.
    """

    def do_not_reevaluate(self):
        """
        Prohibits the next reevaluate.
        This is useful if you have an action which has to be completed without interruption, e.g. a kick animation.
        """
        self._dsd.set_do_not_reevaluate()

    def repr_dict(self):
        """
        Represent this stack element as dictionary which is JSON encodable

        :rtype: dict
        """
        return {
            'type': 'action',
            'classname': self.__class__.__name__,
            'debug_data': self._debug_data
        }
