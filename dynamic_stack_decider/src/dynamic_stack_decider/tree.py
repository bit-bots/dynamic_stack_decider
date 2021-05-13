import sys
from typing import List, Optional

class Tree:
    """ A tree defining a behaviour, parsed from a .dsd file """

    def __init__(self):
        # The root element of the tree
        self.root_element = None
        self.parameter_list = []

    def set_root_element(self, element):
        """
        Set the root element of the tree

        :param element: The root element
        :type element: AbstractTreeElement
        """
        self.root_element = element

    def __repr__(self):
        return repr(self.root_element)


class AbstractTreeElement:
    """
    An element (node) in the tree. Do not use directly,
    use one of DecisionTreeElement and ActionTreeElement instead
    """

    def __init__(self, name, parent):
        self.name = name
        self.parent = parent
        self.module = None
        self.parameters = None
        self.activation_reason = None

    def get_child(self, activating_result):
        # type: (str) -> Optional[AbstractTreeElement]
        """
        Get the child that should be activated for the given result.
        This makes only sense for DecisionTreeElements; others return None

        :param activating_result: The result that activates the returned child
        :return: The corresponding child element or None
        """
        return None

    def set_activation_reason(self, reason):
        """ Set the result that activated this element """
        self.activation_reason = reason


class DecisionTreeElement(AbstractTreeElement):
    """
    A tree element describing a decision. A decision has children that are executed on a certain result.
    Children can be added with add_child_element
    """

    def __init__(self, name, parent, parameters=None, unset_parameters=None):
        """
        Create a new DecisionTreeElement

        :param name: the class name of the corresponding AbstractDecisionElement
        :param parent: the parent element, None for the root element
        :type parent: DecisionTreeElement
        """
        super(DecisionTreeElement, self).__init__(name, parent)
        self.parameters = parameters or dict()
        self.unset_parameters = unset_parameters or dict()

        # Dictionary that maps results of the decision to the corresponding child
        self.children = dict()

    def add_child_element(self, element, activating_result):
        """Add a child that will be executed when activating_result is returned"""
        self.children[activating_result] = element
        element.set_activation_reason(activating_result)

    def get_child(self, activating_result):
        """Get the child for a given result"""
        if not activating_result:
            # give advice about error
            sys.exit("Decision return was None. You probably forgot to return a string in your perform method.")

        if activating_result in self.children.keys():
            return self.children[activating_result]
        elif "ELSE" in self.children.keys():
            return self.children["ELSE"]
        else:
            raise KeyError(f"{activating_result} does not lead to a child of {str(self)} and no ELSE was specified")

    def __repr__(self):
        r = '$' + self.name + ' ({}): '.format(self.parameters)
        for result, child in self.children.items():
            r += result + ': {' + repr(child) + '} '
        return r

    def __str__(self):
        return self.name


class SequenceTreeElement(AbstractTreeElement):
    """
    A tree element describing a sequence of actions. Each action
    has optional parameters that will be passed to the module on
    creation
    """

    def __init__(self, parent):
        super(SequenceTreeElement, self).__init__(None, parent)
        self.action_elements = list()  # type: List[ActionTreeElement]

    def add_action_element(self, action_element):
        """
        Add an action element to the sequence

        :param action_element: The action element to be added
        :type action_element: ActionTreeElement
        """
        action_element.activation_reason = self.activation_reason
        self.action_elements.append(action_element)

    def set_activation_reason(self, reason):
        self.activation_reason = reason
        for action in self.action_elements:
            action.set_activation_reason(reason)

    def __repr__(self):
        return '({})'.format(', '.join(repr(action) for action in self.action_elements))


class ActionTreeElement(AbstractTreeElement):
    """
    A tree element describing an action. An action has optional
    parameters that will be passed to the module on creation
    """

    def __init__(self, name, parent, parameters=None, unset_parameters=None):
        """
        Create a new ActionTreeElement
        :param name: the class name of the corresponding AbstractActionElement
        :param parent: the parent element
        :type parent: DecisionTreeElement
        :param parameters: A dictionary of parameters
        :param unset_parameters: A dictionary of parameters that must be set later
        """
        super(ActionTreeElement, self).__init__(name, parent)
        self.parameters = parameters or dict()
        self.unset_parameters = unset_parameters or dict()
        self.in_sequence = False

    def __repr__(self):
        return '@{} ({})'.format(self.name, self.parameters)

    def __str__(self):
        return self.name
