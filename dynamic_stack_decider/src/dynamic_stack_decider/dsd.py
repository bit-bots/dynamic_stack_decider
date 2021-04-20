import importlib
import inspect
import json
import pkgutil

import rospy
from std_msgs.msg import String
from typing import Dict, List, Tuple, Optional

from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from dynamic_stack_decider.sequence_element import SequenceElement
from dynamic_stack_decider.abstract_stack_element import AbstractStackElement
from dynamic_stack_decider.parser import parse as parse_dsd
from dynamic_stack_decider.tree import Tree, AbstractTreeElement, ActionTreeElement, DecisionTreeElement, \
    SequenceTreeElement


def discover_elements(path):
    """
    Extract all the classes from the files in the given path and return a dictionary containing them

    :param path: The absolute path containing the files that should be registered
    :type path: str
    :return: A dictionary with class names as keys and classes as values
    :rtype: Dict[str, AbstractStackElement]
    """
    elements = {}
    # base_module is the module name that contains the elements to be discovered
    # it is the name relative to the src directory (from where it will be imported)
    base_module = path.split("/src/")[-1].replace('/', '.')

    for _, modname, _ in pkgutil.walk_packages(path=[path], prefix=base_module + '.'):
        try:
            module = importlib.import_module(modname)
            # add all classes which are defined directly in the target module (not imported)
            elements.update(inspect.getmembers(module, lambda m: inspect.isclass(m) and inspect.getmodule(m) == module))
        except Exception as e:
            rospy.logerr('Error while loading class {}: {}'.format(modname, e))
    return elements


class DSD:
    """
    One decision is defined as the root decision, the starting point.
    Each decision element, which is pushed on the stack, is immediately executed until no further element is pushed.
    Following, each iteration, for each element is checked if it requires to be reevaluated and finally the
     top element of the stack will be executed, usually an action.
    If the outcome of a reevaluated element changes, the entire stack on top of this element will be dropped and the
     stack newly constructed.
    As soon as the action is complete, the element will be popped off the stack and the module underneath will be
     executed in the next iteration.
    If this is a decision, it again pushes a further decision or an action and the new top element will be executed.

    By this structure, it is always visible which action the robot tries to perform and which decisions were made.

    If a new element is pushed on top of the stack, it is directly executed.
    In most cases, the pushing element is completing its execution with the push of another element.
    Any following code will be executed as soon as the stack is not further expanded.
    """

    start_element = None
    stack_exec_index = -1
    stack_reevaluate = False
    do_not_reevaluate = False
    old_representation = ""

    def __init__(self, blackboard, debug_topic=None):
        """
        :param blackboard: Blackboard instance which will be available to all modules
        :param debug_topic:  Topic on which debug data should be published
        :type debug_topic: str
        """
        self.blackboard = blackboard

        self.tree = None  # type: Optional[Tree]
        # The stack is implemented as a list of tuples consisting of the tree element
        # and the actual module instance
        self.stack = []  # type: List[Tuple[AbstractTreeElement, AbstractStackElement]]

        self.actions = {}  # type: Dict[str, AbstractActionElement]
        self.decisions = {}  # type: Dict[str, AbstractDecisionElement]

        # Setup debug publisher if needed
        self.debug_active = debug_topic is not None
        if self.debug_active:
            rospy.loginfo('Debugging is active. Publishing on {}'.format(debug_topic))
            self.debug_publisher = rospy.Publisher(debug_topic, String, queue_size=10)

    def register_actions(self, module_path):
        """
        Register every class in a given path as an action
        :param module_path: A path containing files with classes extending AbstractActionElement
        """
        self.actions = discover_elements(module_path)

    def register_decisions(self, module_path):
        """
        Register every class in a given path as a decision
        :param module_path: A path containing files with classes extending AbstractDecisionElement
        """
        self.decisions = discover_elements(module_path)

    def load_behavior(self, path):
        """
        Load a .dsd file into the behaviour to execute it. This should be called after the actions
        and decisions have been loaded.
        :param path: The path to the .dsd file describing the behaviour
        :return:
        """
        self.tree = parse_dsd(path)
        self._bind_modules(self.tree.root_element)
        self.set_start_element(self.tree.root_element)

    def _bind_modules(self, element):
        """
        Recursively traverse the tree and bind the registered action and decision classes to
        the corresponding tree elements
        :param element: The starting element
        """
        if isinstance(element, ActionTreeElement):
            element.module = self.actions[element.name]
        elif isinstance(element, DecisionTreeElement):
            element.module = self.decisions[element.name]
            for child in element.children.values():
                self._bind_modules(child)
        elif isinstance(element, SequenceTreeElement):
            for action in element.action_elements:
                self._bind_modules(action)
        else:
            raise KeyError('Provided element ' + str(element) + 'was not found in registered actions or decisions')

    def _init_element(self, element):
        """ Initialises the module belonging to the given element. """
        if isinstance(element, SequenceTreeElement):
            initialized_actions = list()
            for action in element.action_elements:
                initialized_actions.append(action.module(self.blackboard, self, action.parameters))
            return SequenceElement(self.blackboard, self, initialized_actions)
        else:
            return element.module(self.blackboard, self, element.parameters)

    def set_start_element(self, start_element):
        """
        This method defines the start element on the stack, which stays always on the bottom of the stack.
        It should be called in __init__.
        """
        self.start_element = start_element
        self.stack = [(self.start_element, self._init_element(self.start_element))]

    def interrupt(self):
        """
        An interrupt is an event which clears the complete stack to reset the behavior.
        In the special case of RoboCup, we use it when the game-state changes, but it can also be used for
        example if the robot is kidnapped or paused.
        In the following iteration, the stack will be newly created starting at the root element.
        """
        if self.stack_reevaluate:
            # we were currently checking preconditions
            # we stop this, so that update() knows that it has to stop
            self.stack_reevaluate = False
        self.stack = [(self.start_element, self._init_element(self.start_element))]

    def update(self, reevaluate=True):
        """
        Calls the element which is currently on top of the stack.
        Before doing this, all preconditions are checked (all decision elements where reevaluate is true).

        :param: reevaluate: Can be set to False to avoid the reevaluation
        :type reevaluate: bool
        """
        self.publish_debug_msg()

        if reevaluate and not self.do_not_reevaluate:
            self.stack_exec_index = 0
            self.stack_reevaluate = True
            for tree_element, instance in self.stack[:-1]:
                # check all elements except the top one, but not the actions
                if isinstance(instance, AbstractDecisionElement) and instance.get_reevaluate():
                    result = instance.perform(True)
                    # Push element if necessary
                    if result != self.stack[self.stack_exec_index + 1][0].activation_reason:
                        self.stack = self.stack[0:self.stack_exec_index + 1]
                        self.stack_reevaluate = False
                        self.push(tree_element.get_child(result))

                    if not self.stack_reevaluate:
                        # We had some external interrupt, we stop here
                        return
                self.stack_exec_index += 1
            self.stack_reevaluate = False
        if reevaluate:
            # reset flag
            self.do_not_reevaluate = False
        # run the top module
        current_tree_element, current_instance = self.stack[-1]
        result = current_instance.perform()
        if isinstance(current_instance, AbstractDecisionElement):
            self.push(current_tree_element.get_child(result))

    def push(self, element):
        """
        Put a new element on the stack and start it directly.

        This should only be called by the DSD, not from any of the modules

        :param element: The tree element that should be put on top of the stack.
        :type element: AbstractTreeElement
        """
        self.stack.append((element, self._init_element(element)))

        # we call the new element without another reevaluate
        self.update(False)

    def pop(self):
        """
        Removes the element from the stack. The previous element will not be called again.
        """
        if len(self.stack) > 1:
            if self.stack_reevaluate:
                # we are currently reevaluating. we shorten the stack here
                if self.stack_exec_index > 0:
                    # only shorten stack if it still has one element
                    self.stack = self.stack[0:self.stack_exec_index]
                # stop reevaluating
                self.stack_reevaluate = False
            else:
                if isinstance(self.stack[-1][1], SequenceElement):
                    # If we are in a sequence, only one action should be popped
                    if not self.stack[-1][1].in_last_element():
                        # We are still in the sequence, therefore we do not want to pop the SequenceElement,
                        # only a single element of the sequence
                        # We also do not want to reset do_not_reevaluate because an action in the sequence
                        # may control the stack beyond its own lifetime but in the sequence element's lifetime
                        self.stack[-1][1].pop_one()
                        return
                # Remove the last element of the stack
                self.stack.pop()

            # We will reevaluate even when the popped element set do_not_reevaluate
            # because no module should control the stack beyond its lifetime
            self.do_not_reevaluate = False

    def set_do_not_reevaluate(self):
        """No reevaluation on next iteration"""
        self.do_not_reevaluate = True

    def get_stack(self):
        """
        Returns the current stack
        """
        return self.stack

    def publish_debug_msg(self):
        """
        Helper method to publish debug data
        """

        if self.debug_active:
            # Construct JSON encodable object which represents the current stack
            data = None
            for tree_elem, elem_instance in reversed(self.stack):
                elem_data = elem_instance.repr_dict()
                elem_data['activation_reason'] = tree_elem.activation_reason
                elem_data['next'] = data
                data = elem_data

            msg = String(data=json.dumps(data))
            self.debug_publisher.publish(msg)
