import json
import uuid
from typing import Optional

import pydot
from python_qt_binding.QtGui import QStandardItem, QStandardItemModel
from std_msgs.msg import String
from builtin_interfaces.msg import Time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy


class ParseError(Exception):
    pass


class DsdFollower:
    def __init__(self, node: Node, debug_topic: str):
        """
        Creates a new DSD follower which subscribes to the given debug topics,
        fetches it's tree and listens for the current stack.
        It provides the rendered tree and the current stack visualization based on the received data.

        :param node: Reference to the used ROS node\
        :param debug_topic: The topic namespace on which the DSD publishes its debug data
        """
        self._node = node

        self.dsd_debug_topic = debug_topic

        self.tree: Optional[dict] = None
        self.stack: Optional[dict] = None

        # Subscribe to the DSDs tree (latched)
        self.tree_sub = self._node.create_subscription(
            String,
            f"{debug_topic}/dsd_tree",
            self._tree_callback,
            qos_profile=QoSProfile(
                depth=10,
                reliability=DurabilityPolicy.TRANSIENT_LOCAL))

        self._node.get_logger().info(f"Subscribed to {debug_topic}/dsd_tree")

        # Subscribe to the DSDs stack
        self.stack_sub = self._node.create_subscription(
            String,
            f"{debug_topic}/dsd_stack",
            self._stack_callback,
            10)
        self._node.get_logger().info(f"Subscribed to {debug_topic}/dsd_stack")

        self._cached_dotgraph = None
        self._cached_item_model = None

    def _tree_callback(self, msg):
        self._node.get_logger().info("Received tree")
        self.tree = json.loads(msg.data)

    def _stack_callback(self, msg):
        # Deserialize the stack message
        stack = json.loads(msg.data)

        # Abort if nothing changed
        if self.stack == stack:
            return

        # Abort if no tree was received yet
        if self.tree is None:
            return

        # Update stack
        self.stack = stack
        # Reset cache
        self._cached_dotgraph = None
        self._cached_item_model = None

    @staticmethod
    def _error_dotgraph():
        dot = pydot.Dot(graph_type="digraph")

        uid1 = str(uuid.uuid4())
        dot.add_node(pydot.Node(uid1, label="I have not received anything from the dsd yet"))

        uid2 = str(uuid.uuid4())
        dot.add_node(
            pydot.Node(
                uid2,
                label="Please make sure that\n"
                "- The appropriate dsd is started\n"
                "- You are connected to the same roscore\n"
                "- param /debug_active is True",
            )
        )

        dot.add_edge(pydot.Edge(uid1, uid2))

        return dot

    @staticmethod
    def _empty_item_model():
        return QStandardItemModel()

    @staticmethod
    def _dot_node_from_stack_element(element: dict, active: bool) -> pydot.Node:
        """
        :param element: The element to generate the dot node from
        :param active: Whether the node is currently active or not
        :return: The corresponding dot node
        """

        def param_string(params: dict) -> str:
            """
            :param params: A dict of parameters
            :return: A string representation of the parameters
            """
            # Return empty string if no parameters are given
            if not params: return ""

            output = []
            for param_name, param_value in params.items():
                output.append(f"{param_name}: {str(param_value)}")
            return " (" + ", ".join(output) + ")"


        # Go through all possible element types and create the corresponding label and shape
        if element["type"] == "sequence":
            shape = "box"

            label = ["Sequence:"]
            for i, action in enumerate(element["action_elements"]):
                # Spaces for indentation
                action_label = "  "
                # Mark current element (if this sequence is on the stack)
                if active and i == element["current_action_index"]:
                    action_label += "--> "
                action_label += action["name"] + param_string(action["parameters"])
                label.append(action_label)
            label = "\n".join(label)
        elif element["type"] == "decision":
            shape = "ellipse"
            label = element["name"] + param_string(element["parameters"])

        elif element["type"] == "action":
            shape = "box"
            label = element["name"] + param_string(element["parameters"])
        else:
            raise ParseError(f"Unknown element type {element['type']}")

        # Create node in graph
        return pydot.Node(\
            str(uuid.uuid4()),
            label=label,
            shape=shape,
            color="lightgray" if not active else None,
        )

    def _stack_to_dotgraph(self, dot: pydot.Dot, subtree_root: dict, stack_root: Optional[dict] = None) -> (pydot.Dot, str):
        """
        Recursively modify dot to include every element of the stack
        """
        # Sanity check
        if stack_root is not None:
            assert stack_root["type"] == subtree_root["type"], "The stack and the tree do not match"
            assert stack_root["name"] == subtree_root["name"], "The stack and the tree do not match"

        # Generate dot node for the root and mark it as active if it is on the stack
        dot_node = DsdFollower._dot_node_from_stack_element(subtree_root, stack_root is not None)
        # Append this element to graph
        dot.add_node(dot_node)

        # Append all direct children to graph
        # Also append all children which are on the stack to the graph
        if "children" in subtree_root:
            # Go through all children
            for activating_result, child in subtree_root["children"].items():
                # Get the root of the sub stack if this branch the one which is currently on the stack
                sub_stack_root = None
                if stack_root is not None \
                        and stack_root["next"] is not None \
                        and stack_root["next"]["activation_reason"] == activating_result:
                    sub_stack_root = stack_root["next"]

                # Add this child to the graph
                dot, child_uid = self._stack_to_dotgraph(
                    dot,
                    child,
                    sub_stack_root
                )
                # Connect the child to the parent element
                edge = pydot.Edge(dot_node.get_name(), child_uid, label=activating_result)
                dot.add_edge(edge)
        return dot, dot_node.get_name()

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

        self._node.get_logger().info("Generating dotgraph")

        # Check if we received any data yet
        if self.stack is None or self.tree is None:
            self._node.get_logger().info("No data received yet")
            return self._error_dotgraph()

        # Create dot graph
        dot = pydot.Dot(graph_type="digraph")
        dot, _ = self._stack_to_dotgraph(dot, self.tree, self.stack)

        # Cache result
        self._cached_dotgraph = dot
        return dot

    def to_q_item_model(self):
        """
        Represent the DSDs debug data as QITemModel
        """
        # Return cached result if available
        if self._cached_item_model is not None:
            return self._cached_item_model

        # Return if we received no data yet
        if self.stack is None or self.tree is None:
            return self._empty_item_model()

        return self._empty_item_model() # TODO: Remove this line

        # Construct a new item-model
        model = QStandardItemModel()

        """
        for i in range(len(self.stack)):
            elem, _ = self.stack[i]
            elem_item = QStandardItem()
            elem_item.setEditable(False)

            if isinstance(elem, SequenceTreeElement):
                elem_item.setText("Sequence: " + ", ".join(str(e) for e in elem.action_elements))
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
        """

        self._cached_item_model = model
        return self._cached_item_model

