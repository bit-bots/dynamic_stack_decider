import copy
import re
import rospy
from dynamic_stack_decider.tree import Tree, AbstractTreeElement, DecisionTreeElement, ActionTreeElement, \
    SequenceTreeElement


class DSDParser:
    def parse(self, file):
        """
        Parse a .dsd file to a Tree
        :param file: the path to the .dsd file to be parsed
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
        with open(file, 'r') as bfile:
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
                        element = self.create_tree_element(line_content, current_tree_element)
                        tree.set_root_element(element)
                        current_tree_element = element

                    if indent == 0 and line_content.startswith('#'):
                        # This is the declaration of a new subtree
                        current_subtree = Tree()
                        subtrees[line_content[1:]] = current_subtree
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
                            if subtree_name not in subtrees:
                                raise AssertionError('Error parsing line {}: {} not defined'.format(lnr, call))
                            # The root element of the subtree should be placed in this tree position
                            if current_tree_element is None:
                                # The current subtree is empty, set the subtree as its root element
                                current_subtree.set_root_element(subtrees[subtree_name].root_element)
                            else:
                                # Append this subtree in the current position
                                current_tree_element.add_child_element(copy.copy(subtrees[subtree_name].root_element), result)

                        elif re.search(r'\s*,\s*', call):
                            # A sequence element
                            actions = re.split(r'\s*,\s*', call)
                            element = self.create_sequence_element(actions, current_tree_element)
                            current_tree_element.add_child_element(element, result)

                        elif call.startswith('@'):
                            # An action is called
                            element = self.create_tree_element(call, current_tree_element)
                            current_tree_element.add_child_element(element, result)

                        elif call.startswith('$'):
                            # A decision is called
                            element = self.create_tree_element(call, current_tree_element)
                            current_tree_element.add_child_element(element, result)
                            current_tree_element = element

                        else:
                            raise ParseError('Error parsing line {}: Element {} is neither an action nor a decision'.format(lnr, call))

                    else:
                        # No arrow, must be the beginning of a new subtree
                        element = self.create_tree_element(line_content, current_tree_element)
                        current_subtree.set_root_element(element)
                        current_tree_element = element

                    last_indent = indent
        return tree

    def create_tree_element(self, token, parent):
        """
        Create a tree element given a token and a parent.
        The method derives the type (Action/Decision) and optional parameters from the token
        :param token: the string describing the element in the dsd description
        :param parent: the parent element of the new element, None for root
        :type parent: DecisionTreeElement
        :return: a TreeElement containing the information given in token
        """
        name = token[1:]
        parameters = re.split(r'\s*\+\s*', name)
        name = parameters.pop(0)
        parameter_dict = dict()
        for parameter in parameters:
            parameter_key, parameter_value = parameter.split(':')
            if parameter_value.startswith('%'):
                parameter_value = rospy.get_param(parameter_value[1:])
            parameter_dict[parameter_key] = parameter_value
        if token.startswith('$'):
            element = DecisionTreeElement(name, parent, parameter_dict)
        elif token.startswith('@'):
            element = ActionTreeElement(name, parent, parameter_dict)
        else:
            raise ParseError()
        return element

    def create_sequence_element(self, actions, parent):
        sequence_element = SequenceTreeElement(parent)
        for action in actions:
            element = self.create_tree_element(action, sequence_element)
            if not isinstance(element, ActionTreeElement):
                raise ParseError('In a sequence, only actions are allowed!')
            sequence_element.add_action_element(element)
        return sequence_element


class ParseError(AssertionError):
    pass
