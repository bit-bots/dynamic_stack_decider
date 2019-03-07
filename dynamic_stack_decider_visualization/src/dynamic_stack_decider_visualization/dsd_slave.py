import json
import re
import uuid
import pydot
import rospy
from std_msgs.msg import String
from python_qt_binding.QtGui import QStandardItemModel, QStandardItem
from dynamic_stack_decider.abstract_stack_element import AbstractStackElement
from dynamic_stack_decider.dsd import DSD
from dynamic_stack_decider.tree import AbstractTreeElement, ActionTreeElement, DecisionTreeElement, SequenceTreeElement


class ParseException(Exception):
    pass


class DsdSlave(DSD):

    def __init__(self, debug_topic):
        DSD.__init__(self, None)

        self.debug_subscriber = rospy.Subscriber(debug_topic, String, self.subscriber_callback, queue_size=10)
        self.__cached_msg = None
        self.__cached_dotgraph = None
        self.__cached_item_model = None
        self.initialized = False

    def _init_element(self, element, parameters=None):
        """
        We do not initialize anything on the slave
        """
        return None

    def update(self, reevaluate=True):
        """
        The DSD slave does not execute any code
        """
        pass

    def close(self):
        self.debug_subscriber.unregister()

    def __parse_remote_msg(self, remaining_msg, parent_element):
        """
        Recursively parse the remaining part of a remote DSDs state description message
        :arg parent_element: The Tree element which is the parent of the newly parsed element
        :type parent_element: AbstractTreeElement
        :type remaining_msg: str
        """
        # match the complete expression of one element on the stack
        match_all = re.search('\|-.*?;', remaining_msg)
        if match_all is None:  # end of recursion
            return
        match_all = match_all.group()

        # test edge case
        if isinstance(parent_element, ActionTreeElement):
            raise ParseException('The remote DSD sent further elements which seem to be on the stack'
                                 'but the local DSD tree has already reached an ActionElement')

        match_type = re.search('\$|@|:abstract:', match_all).group()

        # test edge case
        if match_type == ':abstract:':
            raise ParseException('The remote DSD seems to have an abstract element on its stack')

        match_reason = re.search('\|-.*?->', match_all).group()[2:-2]
        match_class = re.search('->..*?\[', match_all).group()[3:-1]
        match_data = re.search('\[.*\];', match_all).group()[1:-2]

        # Search for the correct child element and set it's data
        #   Since the remote DSD should have the same tree it HAS to be directly under the parent_element node
        try:
            element = parent_element.get_child(match_reason)
            element.debug_data = json.loads(match_data)
        except KeyError:
            raise ParseException('The matched activating_result {} could not be found in local tree'
                                 ' (from parent {})'.format(match_reason, parent_element))

        self.push(element)

        self.__parse_remote_msg(remaining_msg.replace(match_all, ''), element)

    def subscriber_callback(self, msg):
        # abort if the dsd is not fully loaded yet
        if not self.initialized:
            return

        msg = msg.data

        # abort if nothing changed
        if msg == self.__cached_msg:
            return
        self.__cached_dotgraph = None
        self.__cached_item_model = None

        # extract the root stack element
        match_root_all = re.search('^\|-None->.*?;', msg).group()
        match_root_data = re.search('\[.*\];', match_root_all).group()[1:-2]
        self.set_start_element(self.tree.root_element)
        self.tree.root_element.debug_data = json.loads(match_root_data)

        # parse the remaining stack (without root element)
        self.__parse_remote_msg(msg.replace(match_root_all, ''), self.start_element)

        # save the message so we know not to reprocess it again
        self.__cached_msg = msg

    @staticmethod
    def __error_dotgraph():
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
    def __empty_item_model():
        return QStandardItemModel()

    def __stack_to_dotgraph(self, stack, dot):
        """
        Recursively modify dot to include every element of the stack
        """
        element, _ = stack[0]

        # Determine correct shape
        if isinstance(element, SequenceTreeElement):
            shape = 'ellipse'
            element.name = 'Sequence: ' + ', '.join([e.name for e in element.action_elements])
        elif isinstance(element, DecisionTreeElement):
            shape = 'box'
        else:
            shape = 'ellipse'

        # Create node in graph
        uid = str(uuid.uuid4())
        node = pydot.Node(uid, label=element.name, shape=shape)
        dot.add_node(node)

        # Append all direct children to graph
        # Also append all children which are on the stack to the graph
        if isinstance(element, DecisionTreeElement):
            for activating_result, child in element.children.items():

                # Since this child is on the stack as well, it should be represented completely
                if len(stack) > 1 and activating_result == stack[1][0].activation_reason:
                    dot, child_uid = self.__stack_to_dotgraph(stack[1:], dot)

                # Draw this child as shape because we want to show direct children of elements
                else:
                    # Determine shape of child
                    if isinstance(child, SequenceTreeElement):
                        child_shape = 'ellipse'
                        child.name = 'Sequence: ' + ', '.join([e.name for e in child.action_elements])
                    elif isinstance(child, DecisionTreeElement):
                        child_shape = 'box'
                    else:
                        child_shape = 'ellipse'

                    # Create child node in graph
                    child_uid = str(uuid.uuid4())
                    child_node = pydot.Node(child_uid, label=child.name, shape=child_shape, color='lightgray')
                    dot.add_node(child_node)

                # Connect the child to the parent element
                edge = pydot.Edge(uid, child_uid, label=activating_result)
                dot.add_edge(edge)

        return dot, uid

    def __append_element_to_item(self, parent_item, debug_data):
        """
        Append an elements debug_data to a QStandardItem.

        :type parent_item: python_qt_binding.QtGui.QStandardItem
        :type debug_data: dict or list or int or float or str or bool
        :rtype: python_qt_binding.QtGui.QStandardItem
        """
        if type(debug_data) is list:
            for i, data in enumerate(debug_data):
                child_item = QStandardItem()
                child_item.setText(str(i) + ": ")
                child_item.setEditable(False)
                self.__append_element_to_item(child_item, data)
                parent_item.appendRow(child_item)
        elif type(debug_data) is dict:
            for label, data in debug_data.items():
                child_item = QStandardItem()
                child_item.setText(str(label) + ": ")
                child_item.setEditable(False)
                self.__append_element_to_item(child_item, data)
                parent_item.appendRow(child_item)
        elif type(debug_data) is str or type(debug_data) is int \
                or type(debug_data) is float or type(debug_data) is bool\
                or type(debug_data) is unicode:
            parent_item.setText(parent_item.text() + str(debug_data))

    def to_dotgraph(self):
        """
        Represent the current stack as dotgraph
        """
        # Return cached result if available
        if self.__cached_dotgraph is not None:
            return self.__cached_dotgraph

        # Return special error graph which shows error information when no data was received
        if self.__cached_msg is None:
            return self.__error_dotgraph()

        dot = pydot.Dot(graph_type='digraph')
        dot, uid = self.__stack_to_dotgraph(self.stack, dot)

        self.__cached_dotgraph = dot
        return dot

    def to_QItemModel(self):
        """
        Represent the DSDs debug data as QITemModel
        """
        # Return cached result if available
        if self.__cached_item_model is not None:
            return self.__cached_item_model

        # Return empty model when no dsd data was received yet
        if self.__cached_msg is None:
            return self.__empty_item_model()

        # Construct a new item-model
        model = QStandardItemModel()
        for elem, _ in self.stack:
            elem_item = QStandardItem()
            elem_item.setText(str(elem))
            elem_item.setEditable(False)
            self.__append_element_to_item(elem_item, elem.debug_data)
            model.invisibleRootItem().appendRow(elem_item)

            # Add a spacer if this is not the last item
            if elem != self.stack[-1][0]:
                spacer = QStandardItem()
                spacer.setEditable(False)
                model.invisibleRootItem().appendRow(spacer)

        self.__cached_item_model = model
        return self.__cached_item_model
