import json
import uuid
import pydot
import rospy
from std_msgs.msg import String
from python_qt_binding.QtGui import QStandardItemModel, QStandardItem
from dynamic_stack_decider.dsd import DSD
from dynamic_stack_decider.tree import AbstractTreeElement, ActionTreeElement, DecisionTreeElement, SequenceTreeElement


class ParseException(Exception):
    pass


class DsdFollower(DSD):

    def __init__(self, debug_topic):
        super().__init__(None)

        self.debug_subscriber = rospy.Subscriber(debug_topic, String, self.subscriber_callback, queue_size=10)
        self._cached_msg = None
        self._cached_dotgraph = None
        self._cached_item_model = None
        self.initialized = False

    def _init_element(self, element, parameters=None):
        """
        We do not initialize anything on the follower
        """
        return None

    def update(self, reevaluate=True):
        """
        The DSD follower does not execute any code
        """
        pass

    def close(self):
        self.debug_subscriber.unregister()

    def _parse_remote_data(self, remaining_data, parent_element=None):
        """
        Recursively parse the remaining part of a remote DSDs state description message
        :arg parent_element: The Tree element which is the parent of the newly parsed element
        :type parent_element: AbstractTreeElement
        :type remaining_data: dict
        """
        if remaining_data is None:
            # recursion exit
            return

        if remaining_data['type'] == 'abstract':
            raise ParseException('Remote DSD sent an abstract element in its stack')

        if isinstance(parent_element, ActionTreeElement):
            raise ParseException('The remote DSD sent further elements which seem to be on the stack'
                                 'but the local DSD tree has already reached an ActionElement')

        if parent_element is None:
            # If no parent_element was given, then we are processing the root element
            self.set_start_element(self.tree.root_element)
            self.tree.root_element.debug_data = remaining_data['debug_data']

            self._parse_remote_data(remaining_data['next'], self.tree.root_element)

        else:
            element = parent_element.get_child(remaining_data['activation_reason'])
            element.debug_data = remaining_data['debug_data']
            if remaining_data['type'] == 'sequence':
                element.current_child = remaining_data['current_action_id']
                self.push(element)
            else:
                self.push(element)
                self._parse_remote_data(remaining_data['next'], element)

    def subscriber_callback(self, msg):
        # abort if the dsd is not fully loaded yet
        if not self.initialized:
            return

        msg = msg.data

        # abort if nothing changed
        if msg == self._cached_msg:
            return
        self._cached_dotgraph = None
        self._cached_item_model = None

        # parse the remaining stack (without root element)
        self._parse_remote_data(json.loads(msg))

        # save the message so we know not to reprocess it again
        self._cached_msg = msg

    @staticmethod
    def _error_dotgraph():
        dot = pydot.Dot(graph_type='digraph')

        param_debug_active = rospy.get_param("/debug_active", False)

        uid1 = str(uuid.uuid4())
        dot.add_node(pydot.Node(uid1, label="I have not received anything from the dsd yet"))

        uid2 = str(uuid.uuid4())
        dot.add_node(pydot.Node(uid2, label="Please make sure that\n"
                                            "- The appropriate dsd is started\n"
                                            "- You are connected to the same roscore\n"
                                            "- param /debug_active is True (for me it is {})".format(param_debug_active)))

        dot.add_edge(pydot.Edge(uid1, uid2))

        return dot

    @staticmethod
    def _empty_item_model():
        return QStandardItemModel()

    @staticmethod
    def _dot_node_from_stack_element(element, active):
        """
        :param element: The element to generate the dot node from
        :type element: AbstractTreeElement
        :param active: Whether the node is currently active or not
        :type active: bool
        :return: The corresponding dot node
        :rtype: pydot.Node
        """

        def param_string(params):
            # type: (dict) -> str
            pstr = []
            for pkey, pval in params.items():
                pstr.append(pkey + ': ' + str(pval))
            pstr = ', '.join(pstr)
            pstr = ' (' + pstr + ')'
            return pstr

        if isinstance(element, SequenceTreeElement):
            shape = 'box'

            label = ['Sequence:']
            for i, e in enumerate(element.action_elements):
                # Spaces for indentation
                action_label = '  '
                # Mark current element (if this sequence is on the stack)
                if active and i == element.current_child:
                    action_label += '--> '
                action_label += e.name
                if e.parameters:
                    action_label += param_string(e.parameters)
                label.append(action_label)
            label = '\n'.join(label)

        elif isinstance(element, DecisionTreeElement):
            shape = 'ellipse'
            label = element.name
            if element.parameters:
                label += param_string(element.parameters)

        else:
            shape = 'box'
            label = element.name
            if element.parameters:
                label += param_string(element.parameters)

        # Create node in graph
        uid = str(uuid.uuid4())
        if active:
            return pydot.Node(uid, label=label, shape=shape)
        else:
            return pydot.Node(uid, label=label, shape=shape, color='lightgray')

    def _stack_to_dotgraph(self, stack, dot):
        """
        Recursively modify dot to include every element of the stack
        """
        element, _ = stack[0]

        node = DsdFollower._dot_node_from_stack_element(element, True)

        # Determine correct shape
        dot.add_node(node)

        # Append all direct children to graph
        # Also append all children which are on the stack to the graph
        if isinstance(element, DecisionTreeElement):
            for activating_result, child in element.children.items():

                # Since this child is on the stack as well, it should be represented completely
                if len(stack) > 1 and activating_result == stack[1][0].activation_reason:
                    dot, child_uid = self._stack_to_dotgraph(stack[1:], dot)

                # Draw this child as shape because we want to show direct children of elements
                else:
                    child_node = DsdFollower._dot_node_from_stack_element(child, False)
                    child_uid = child_node.get_name()
                    dot.add_node(child_node)

                # Connect the child to the parent element
                edge = pydot.Edge(node.get_name(), child_uid, label=activating_result)
                dot.add_edge(edge)

        return dot, node.get_name()

    def _append_element_to_item(self, parent_item, debug_data):
        """
        Append an elements debug_data to a QStandardItem.

        :type parent_item: python_qt_binding.QtGui.QStandardItem
        :type debug_data: dict or list or int or float or str or bool
        :rtype: python_qt_binding.QtGui.QStandardItem
        """
        if isinstance(debug_data, list):
            for i, data in enumerate(debug_data):
                child_item = QStandardItem()
                child_item.setText(str(i) + ": ")
                child_item.setEditable(False)
                self._append_element_to_item(child_item, data)
                parent_item.appendRow(child_item)
        elif isinstance(debug_data, dict):
            for label, data in debug_data.items():
                child_item = QStandardItem()
                child_item.setText(str(label) + ": ")
                child_item.setEditable(False)
                self._append_element_to_item(child_item, data)
                parent_item.appendRow(child_item)
        elif isinstance(debug_data, (bool, float, int, str, bytes)):
            parent_item.setText(parent_item.text() + str(debug_data))

    def to_dotgraph(self):
        """
        Represent the current stack as dotgraph
        """
        # Return cached result if available
        if self._cached_dotgraph is not None:
            return self._cached_dotgraph

        # Return special error graph which shows error information when no data was received
        if self._cached_msg is None:
            return self._error_dotgraph()

        dot = pydot.Dot(graph_type='digraph')
        dot, uid = self._stack_to_dotgraph(self.stack, dot)

        self._cached_dotgraph = dot
        return dot

    def to_QItemModel(self):
        """
        Represent the DSDs debug data as QITemModel
        """
        # Return cached result if available
        if self._cached_item_model is not None:
            return self._cached_item_model

        # Return empty model when no dsd data was received yet
        if self._cached_msg is None:
            return self._empty_item_model()

        # Construct a new item-model
        model = QStandardItemModel()
        for i in range(len(self.stack)):
            elem, _ = self.stack[i]
            elem_item = QStandardItem()
            elem_item.setEditable(False)

            if isinstance(elem, SequenceTreeElement):
                elem_item.setText('Sequence: ' + ', '.join(str(e) for e in elem.action_elements))
                sequence = True
            else:
                elem_item.setText(str(elem))
                sequence = False

            self._append_element_to_item(elem_item, elem.debug_data)

            model.invisibleRootItem().appendRow(elem_item)

            # Add a spacer if this is not the last item
            if elem != self.stack[-1][0]:
                spacer = QStandardItem()
                spacer.setEditable(False)
                model.invisibleRootItem().appendRow(spacer)

            if sequence:
                break

        self._cached_item_model = model
        return self._cached_item_model
