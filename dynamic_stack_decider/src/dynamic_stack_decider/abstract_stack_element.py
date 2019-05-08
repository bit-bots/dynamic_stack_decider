import rospy


class AbstractStackElement(object):
    """
    The AbstractStackElement is the basis of all elements on the stack.
    It provides some help functions which should not be overloaded.
    The work of an element is done in the :func:`perform`.
    Each element which inheritaces the AbstractStackElement can be used as a root element on the stack.
    """
    dsd = None
    _init_data = None

    def __init__(self, blackboard, dsd, parameters=None):
        """
        :param blackboard: Shared blackboard for data exchange between elements
        :param dsd: The stack decider which has this element on its stack.
        :param parameters: Optional parameters which serve as arguments to this element
        """
        self._debug_data = {}
        '''This is a dict in which data can be saved that should get represented on a __repr__ call'''

        self.dsd = dsd
        self.blackboard = blackboard

    def setup_internals(self, behaviour):
        """
        This method initilizes the internal variables and gets called by the stack machine.


        """

    def pop(self):
        """
        Help method which pops the element of the stack.

        This method should always be called with a return::
            return self.pop()

        If no return is used, further code is executed after the pop, which leads to difficult to debug behavior.

        """
        self.dsd.pop()

    def perform(self, reevaluate=False):
        """
        This method is called when the element is on top of the stack.
        This method has to be overloaded by the implementation!

        :param reevaluate: True if the current method call is a reevaluate of the state. Meaning the modul is not on top of the stack.
        """
        msg = "You should override perform() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def interrupt(self):
        """
        An interrupt leads to a complete clearing of the stack.
        """
        self.dsd.interrupt()

    def publish_debug_data(self, label, data):
        """
        Publish debug data. Can be viewed using the dsd-visualization

        This method is safe to call without wrapping it in a try-catch block although invalid values will
        be wrapped in a `str()` call

        :type label: str
        :type data: dict or list or int or float or str or bool
        """
        if type(data) not in (dict, list, int, float, str, bool):
            rospy.logdebug_throttle(1, "The supplied debug data of type {} is not JSON serializable and will be wrapped in str()".format(type(data)))
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
        """
        Represent this stack element as dictionary which is JSON encodable

        :rtype: dict
        """
        return {
            'type': 'abstract',
            'classname': self.__class__.__name__,
            'debug_data': self._debug_data
        }

    @staticmethod
    def sign(x):
        return -1 if x < 0 else 1
