import rospy
from abc import ABCMeta, abstractmethod
from typing import Union


class AbstractStackElement(metaclass=ABCMeta):
    """
    The AbstractStackElement is the basis of all elements on the stack.
    It provides some help functions which should not be overloaded.
    The work of an element is done in the :func:`perform`.
    Each element which inherits from the AbstractStackElement can be used as a root element on the stack.
    """
    _dsd = None
    _init_data = None

    def __init__(self, blackboard, dsd, parameters=None):
        """
        :param blackboard: Shared blackboard for data exchange between elements
        :param dsd: The stack decider which has this element on its stack.
        :param parameters: Optional parameters which serve as arguments to this element
        """
        self._debug_data = {}
        """
        This is a dict in which data can be saved that should get represented on a __repr__ call.
        It can be set using publish_debug_data()
        """

        self._dsd = dsd
        self.blackboard = blackboard

    def pop(self):
        """
        Help method which pops the element of the stack.

        This method should always be called with a return:
            return self.pop()

        If no return is used, further code is executed after the pop, which leads to difficult to debug behavior.
        """
        self._dsd.pop()

    @abstractmethod
    def perform(self, reevaluate=False):
        """
        This method is called when the element is on top of the stack.
        This method has to be overloaded by the implementation!

        :param reevaluate: True if the current method call is a reevaluate of the state. Meaning the module is not on top of the stack.
        """
        msg = "You should override perform() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def interrupt(self):
        """
        An interrupt leads to a complete clearing of the stack.
        """
        self._dsd.interrupt()

    def publish_debug_data(self, label, data):
        # type: (str, Union[dict, list, int, float, str, bool])
        """
        Publish debug data. Can be viewed using the DSD visualization

        This method is safe to call without wrapping it in a try-catch block although invalid values will
        be wrapped in a `str()` call

        :param label: A label that describes the given data
        :param data: data that should be displayed for debugging purposes
        """
        if type(data) not in (dict, list, int, float, str, bool):
            rospy.logdebug_throttle(1, "The supplied debug data of type {} is not JSON"
                                       "serializable and will be wrapped in str()".format(type(data)))
            data = str(data)

        rospy.logdebug('{}: {}'.format(label, data))
        self._debug_data[label] = data

    def clear_debug_data(self):
        """
        Clear existing debug data

        This is needed when old values are no longer supposed to be visible
        """
        self._debug_data = {}

    def repr_dict(self):
        # type: () -> dict
        """ Represent this stack element as dictionary which is JSON encodable """
        return {
            'type': 'abstract',
            'classname': self.__class__.__name__,
            'debug_data': self._debug_data
        }
