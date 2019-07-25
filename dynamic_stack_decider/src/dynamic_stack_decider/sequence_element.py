from dynamic_stack_decider.abstract_stack_element import AbstractStackElement
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class SequenceElement(AbstractStackElement):
    """
    A sequence element contains multiple action elements.
    The sequence element executes all the actions consecutively. That means that the first action will be performed
    until it pops itself off the stack. Then, the second action is performed. When the last action is popped, the
    sequence element pops itself too.
    This is not an abstract class to inherit from.
    """
    def __init__(self, blackboard, dsd, actions=()):
        """
        :param actions: list of initialized action elements
        """
        super(SequenceElement, self).__init__(blackboard, dsd)
        self.actions = actions
        self.current_action_index = 0

    def perform(self, reevaluate=False):
        self.current_action.perform()

    def pop_one(self):
        """
        Pop a single element of the sequence
        :return: True when the pop was successful, False when the end of the sequence has been reached
        """
        self.current_action_index += 1
        if self.current_action_index == len(self.actions):
            return False
        else:
            return True

    @property
    def current_action(self):
        """
        :return: The currently executed action of the sequence element
        :rtype: AbstractActionElement
        """
        return self.actions[self.current_action_index]

    def repr_dict(self):
        """
        Represent this stack element as dictionary which is JSON encodable

        :rtype: dict
        """
        self.clear_debug_data()
        self.publish_debug_data('Active Element', self.current_action.__class__.__name__)
        if self.current_action._debug_data:
            self.publish_debug_data('Corresponding debug data', self.current_action._debug_data)
        return {
            'type': 'sequence',
            'current': self.current_action.__class__.__name__,
            'content': [elem.repr_dict() for elem in self.actions],
            'debug_data': self._debug_data
        }
