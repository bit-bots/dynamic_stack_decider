import copy
import re
from dynamic_stack_decider.tree import Tree, AbstractTreeElement, DecisionTreeElement, ActionTreeElement, \
    SequenceTreeElement
import yaml
import rospy
from typing import List, Union


def parse(file_path):
    """
    Parse a .dsd file to a Tree

    :param file_path: the path to the .dsd file to be parsed
    :return: a Tree object containing the parsed elements
    """
    # The root tree
    tree = Tree()
    # Dictionary of subtrees that are created
    # The key is the name and the value is the corresponding TreeElement
    subtrees = dict()

    current_subtree = tree
    current_tree_element = None
    next_is_start = False
    next_is_comment = False
    comment = False
    last_indent = 0
    lnr = 0
    with open(file_path, 'r') as bfile:
        for line in bfile:
            lnr += 1
            comment = next_is_comment

            line = re.sub(r'//\*\*.*?\*\*//', '', line)  # Block comments starting and ending in the same line

            if '**//' in line:
                # Block comments ending in this line
                # This line as well as the following will contain valid code
                next_is_comment = False
                comment = False
                line = re.sub(r'.*\*\*//', '', line)
            if '//**' in line:
                # Block comments starting in this line
                # This line may contain valid code, the next ones won't
                next_is_comment = True
                line = re.sub(r'//\*\*.*', '', line)

            line = re.sub(r'//.*', '', line)  # Line comments

            line = line.rstrip()
            if not line:
                continue

            if not comment:
                indent = len(line) - len(line.lstrip())
                if indent % 4 != 0:
                    raise ParseError('Error parsing line {}: Indent is not a multiple of 4'.format(lnr))

                line_content = line.lstrip()

                if indent == 0 and line_content.startswith('-->'):
                    # This is the declaration of the start. Next line contains root element
                    next_is_start = True
                    current_subtree = tree
                    last_indent = indent
                    continue

                if next_is_start:
                    # This line contains the root element of the main tree
                    next_is_start = False
                    element = _create_tree_element(line_content, current_tree_element, lnr)
                    tree.set_root_element(element)
                    current_tree_element = element

                if indent == 0 and line_content.startswith('#'):
                    # This is the declaration of a new subtree
                    current_subtree = Tree()
                    name = line_content[1:]
                    parameter_list = re.split(r'\s*\+\s*', name)
                    name = parameter_list.pop(0)
                    current_subtree.parameter_list = parameter_list
                    subtrees[name] = current_subtree
                    current_tree_element = None
                    last_indent = indent
                    continue

                if indent < last_indent:
                    # Go layers up, depending on indent difference
                    for _ in range(indent, last_indent, 4):
                        current_tree_element = current_tree_element.parent

                if re.search(r'\s*-?->\s*', line_content):
                    # Arrow in line, split in decision result and call
                    result, call = re.split(r'\s*-?->\s*', line_content, 1)

                    if call.startswith('#'):
                        # A subtree is called here.
                        subtree_name = call.strip('#')
                        name, parameters, unset_parameters = _extract_parameters(subtree_name, lnr)
                        if name not in subtrees:
                            raise ParseError('Error parsing line {}: {} not defined'.format(lnr, name))
                        subtree = subtrees[name]
                        if (set(parameters.keys()) | set(unset_parameters.keys())) != set(subtree.parameter_list):
                            raise ParseError('Error parsing line {}: Invalid parameters specified.\n'
                                             'Available parameters are {}, specified parameters are {}.'
                                             .format(lnr,
                                                     set(parameters.keys()) | set(unset_parameters.keys()),
                                                     set(subtree.parameter_list)))
                        # The root element of the subtree should be placed in this tree position
                        if current_tree_element is None:
                            # The current subtree is empty, set the subtree as its root element
                            current_subtree.set_root_element(subtree.root_element)
                        else:
                            # Copy this subtree
                            subtree_copy = copy.deepcopy(subtree.root_element)
                            # Evaluate all unset parameters
                            to_evaluate = [subtree_copy]
                            while len(to_evaluate) > 0:
                                current = to_evaluate.pop(0)
                                if isinstance(current, SequenceTreeElement):
                                    # For sequence elements, only evaluate the actions
                                    to_evaluate.extend(current.action_elements)
                                    continue
                                elif isinstance(current, DecisionTreeElement):
                                    to_evaluate.extend(current.children.values())

                                for name, reference in current.unset_parameters.items():
                                    if reference in parameters:
                                        current.parameters[name] = parameters[reference]
                                    elif reference in unset_parameters:
                                        current.unset_parameters[name] = unset_parameters[reference]
                                    else:
                                        raise ParseError('Error evaluating subtree call in line {}: '
                                                         'Unknown reference to {}.'.format(lnr, reference))

                            # Append this subtree in the current position
                            current_tree_element.add_child_element(subtree_copy, result)

                    elif re.search(r'\s*,\s*', call):
                        # A sequence element
                        actions = re.split(r'\s*,\s*', call)
                        element = _create_sequence_element(actions, current_tree_element, lnr)
                        current_tree_element.add_child_element(element, result)

                    elif call.startswith('@'):
                        # An action is called
                        element = _create_tree_element(call, current_tree_element, lnr)
                        current_tree_element.add_child_element(element, result)

                    elif call.startswith('$'):
                        # A decision is called
                        element = _create_tree_element(call, current_tree_element, lnr)
                        current_tree_element.add_child_element(element, result)
                        current_tree_element = element

                    else:
                        raise ParseError(
                            'Error parsing line {}: '
                            'Element {} is neither an action nor a decision'.format(lnr, call))

                else:
                    # No arrow, must be the beginning of a new subtree
                    element = _create_tree_element(line_content, current_tree_element, lnr)
                    current_subtree.set_root_element(element)
                    current_tree_element = element

                last_indent = indent
    return tree


def _extract_parameters(token, lnr):
    """
    Extract parameters from a token string in the form of name + key1:value1 + key2:value2
    :param token: the string containing the name and the parameters
    :type token: str
    :param lnr: Line number of the current line (used for error messages)
    :type lnr: int
    :return: the name, a dict of set parameters, a dict of unset parameters
    """
    parameters = re.split(r'\s*\+\s*', token)
    name = parameters.pop(0)
    parameter_dict = dict()
    unset_parameters = dict()
    for parameter in parameters:
        try:
            parameter_key, parameter_value = parameter.split(':')
        except ValueError:
            raise ParseError('Error parsing line {}: Invalid parameter list'.format(lnr))

        if parameter_value.startswith('%'):
            parameter_value = rospy.get_param(parameter_value[1:])
            parameter_dict[parameter_key] = parameter_value
        elif parameter_value.startswith('*'):
            # This is a reference to the value specified in the subtree
            unset_parameters[parameter_key] = parameter_value[1:]
        else:
            parameter_value = yaml.safe_load(parameter_value)  # universal interpretation of correct datatype
            parameter_dict[parameter_key] = parameter_value
    return name, parameter_dict, unset_parameters


def _create_tree_element(token, parent, lnr):
    """
    Create a tree element given a token and a parent.
    The method derives the type (Action/Decision) and optional parameters from the token
    :param token: the string describing the element in the dsd description
    :param parent: the parent element of the new element, None for root
    :type parent: Union[DecisionTreeElement, SequenceTreeElement]
    :param lnr: Line number of the current line (used for error messages)
    :type lnr: int
    :return: a TreeElement containing the information given in token
    """
    name, parameter_dict, unset_parameters = _extract_parameters(token[1:], lnr)
    if token.startswith('$'):
        element = DecisionTreeElement(name, parent, parameter_dict, unset_parameters)
    elif token.startswith('@'):
        element = ActionTreeElement(name, parent, parameter_dict, unset_parameters)
    else:
        raise ParseError()
    return element


def _create_sequence_element(actions, parent, lnr):
    """
    Create a new sequence element

    :param actions: The names of actions in the sequence
    :type actions: List[str]
    :param parent: The parent element of the sequence
    :type parent: AbstractDecisionElement
    :param lnr: Line number of the current line (used for error messages)
    :type lnr: int
    :return: The sequence element
    """
    sequence_element = SequenceTreeElement(parent)
    for action in actions:
        element = _create_tree_element(action, sequence_element, lnr)
        if not isinstance(element, ActionTreeElement):
            raise ParseError('In a sequence, only actions are allowed!')
        sequence_element.add_action_element(element)
    return sequence_element


class ParseError(AssertionError):
    """An error during the DSD file parsing occurred"""
    pass
