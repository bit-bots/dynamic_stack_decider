"""Receive DSD debug snapshots and render them for RQT."""

from __future__ import annotations

import json
import threading
from typing import Literal, cast

import pydot
from dynamic_stack_decider.types import StackData, TreeData
from PyQt5.QtGui import QStandardItem, QStandardItemModel
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.subscription import Subscription
from std_msgs.msg import String


class ParseError(ValueError):
    """A debug message does not match the DSD snapshot schema."""


def _mapping(value: object, label: str) -> dict[str, object]:
    if not isinstance(value, dict) or any(not isinstance(key, str) for key in value):
        raise ParseError(f"{label} must be an object with string keys")
    return cast(dict[str, object], value)


def _identity(data: dict[str, object]) -> tuple[int, str, Literal["action", "decision"]]:
    identifier, name, kind = data.get("id"), data.get("name"), data.get("type")
    if not isinstance(identifier, int) or isinstance(identifier, bool) or not isinstance(name, str):
        raise ParseError("Element id must be an integer and name must be a string")
    if kind == "action":
        return identifier, name, "action"
    if kind == "decision":
        return identifier, name, "decision"
    raise ParseError(f"Unknown element type {kind!r}")


def _parse_tree(value: object) -> TreeData:
    data = _mapping(value, "Tree element")
    identifier, name, kind = _identity(data)
    tree: TreeData = {
        "id": identifier,
        "name": name,
        "type": kind,
        "parameters": _mapping(data.get("parameters"), "Parameters"),
    }
    if kind == "decision":
        tree["children"] = {
            key: _parse_tree(child) for key, child in _mapping(data.get("children"), "Children").items()
        }
    else:
        successor = data.get("next")
        tree["next"] = _parse_tree(successor) if successor is not None else None
    return tree


def _parse_stack(value: object) -> StackData | None:
    if value is None:
        return None
    data = _mapping(value, "Stack element")
    identifier, name, kind = _identity(data)
    reason = data.get("activation_reason")
    if reason is not None and not isinstance(reason, str):
        raise ParseError("Activation reason must be a string or null")
    if "next" not in data:
        raise ParseError("Stack element is missing its next field")
    return {
        "id": identifier,
        "name": name,
        "type": kind,
        "activation_reason": reason,
        "debug_data": _mapping(data.get("debug_data"), "Debug data"),
        "next": _parse_stack(data["next"]),
    }


class DsdFollower:
    def __init__(self, node: Node, debug_topic: str) -> None:
        self._node = node
        self.dsd_debug_topic = debug_topic
        self.tree: TreeData | None = None
        self.stack: StackData | None = None
        self._tree_by_id: dict[int, TreeData] = {}
        self._lock = threading.RLock()
        self._cached_dotgraph: pydot.Dot | None = None
        self._cached_full_tree: bool | None = None
        self._cached_item_model: QStandardItemModel | None = None
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tree_sub: Subscription[String] | None = node.create_subscription(
            String, f"{debug_topic}/dsd_tree", self._tree_callback, latched
        )
        self.stack_sub: Subscription[String] | None = node.create_subscription(
            String, f"{debug_topic}/dsd_stack", self._stack_callback, 10
        )

    def _tree_callback(self, msg: String) -> None:
        try:
            tree = _parse_tree(json.loads(msg.data))
            index: dict[int, TreeData] = {}
            pending = [tree]
            while pending:
                element = pending.pop()
                if element["id"] in index:
                    raise ParseError("Tree element ids must be unique")
                index[element["id"]] = element
                pending.extend(element.get("children", {}).values())
                successor = element.get("next")
                if successor is not None:
                    pending.append(successor)
        except (ValueError, RecursionError) as error:
            self._node.get_logger().error(f"Invalid DSD tree: {error}")
            return
        with self._lock:
            if tree != self.tree:
                self.tree = tree
                self._tree_by_id = index
                self.stack = None
                self._cached_item_model = None
                self.reset_cache()

    def _stack_callback(self, msg: String) -> None:
        try:
            stack = _parse_stack(json.loads(msg.data))
        except (ValueError, RecursionError) as error:
            self._node.get_logger().error(f"Invalid DSD stack: {error}")
            return
        with self._lock:
            if self.tree is None:
                return
            frame = stack
            while frame is not None:
                tree_element = self._tree_by_id.get(frame["id"])
                if tree_element is None:
                    self._node.get_logger().debug("Ignoring stack from a different behavior tree")
                    return
                if frame["name"] != tree_element["name"] or frame["type"] != tree_element["type"]:
                    self._node.get_logger().error("DSD stack identity does not match its tree")
                    return
                frame = frame["next"]
            if self.stack_dict_changed(self.stack, stack, ignore_keys=["debug_data"]):
                self.reset_cache()
            if stack != self.stack:
                self._cached_item_model = None
            self.stack = stack

    def reset_cache(self) -> None:
        with self._lock:
            self._cached_dotgraph = None
            self._cached_full_tree = None

    @staticmethod
    def stack_dict_changed(old_stack_element: object, new_stack_element: object, ignore_keys: list[str]) -> bool:
        if type(old_stack_element) is not type(new_stack_element):
            return True
        if isinstance(old_stack_element, dict) and isinstance(new_stack_element, dict):
            old_keys = old_stack_element.keys() - set(ignore_keys)
            new_keys = new_stack_element.keys() - set(ignore_keys)
            return old_keys != new_keys or any(
                DsdFollower.stack_dict_changed(old_stack_element[key], new_stack_element[key], ignore_keys)
                for key in old_keys
            )
        if isinstance(old_stack_element, list) and isinstance(new_stack_element, list):
            return len(old_stack_element) != len(new_stack_element) or any(
                DsdFollower.stack_dict_changed(old, new, ignore_keys)
                for old, new in zip(old_stack_element, new_stack_element, strict=True)
            )
        if isinstance(old_stack_element, (int, float, str, bool)) or old_stack_element is None:
            return old_stack_element != new_stack_element
        raise ParseError(f"Unknown debug value type {type(old_stack_element).__name__}")

    @staticmethod
    def _error_dotgraph() -> pydot.Dot:
        dot = pydot.Dot(graph_type="digraph")
        dot.add_node(
            pydot.Node(
                "waiting",
                label="Waiting for a DSD tree and stack.\nCheck the selected topic and ROS 2 discovery configuration.",
            )
        )
        return dot

    @staticmethod
    def _dot_node_from_tree_element(tree_element: TreeData, stack_element: StackData | None = None) -> pydot.Node:
        parameters = tree_element["parameters"]
        suffix = " (" + ", ".join(f"{key}: {value}" for key, value in parameters.items()) + ")" if parameters else ""
        return pydot.Node(
            str(tree_element["id"]),
            shape="ellipse" if tree_element["type"] == "decision" else "box",
            label=tree_element["name"] + suffix,
            color="orangered" if stack_element is not None else "gray",
        )

    @staticmethod
    def _stack_matches_tree(stack_element: StackData | None, tree_element: TreeData) -> bool:
        return stack_element is not None and stack_element["id"] == tree_element["id"]

    def _stack_to_dotgraph(
        self, dot: pydot.Dot, subtree_root: TreeData, stack_root: StackData | None = None, full_tree: bool = False
    ) -> tuple[pydot.Dot, str]:
        node_stack = stack_root if self._stack_matches_tree(stack_root, subtree_root) else None
        node = self._dot_node_from_tree_element(subtree_root, node_stack)
        dot.add_node(node)
        if node_stack is not None or full_tree:
            for outcome, child in subtree_root.get("children", {}).items():
                child_stack = None
                if node_stack is not None and node_stack["next"] is not None:
                    if node_stack["next"]["activation_reason"] == outcome:
                        child_stack = node_stack["next"]
                dot, child_id = self._stack_to_dotgraph(dot, child, child_stack, full_tree)
                dot.add_edge(pydot.Edge(node.get_name(), child_id, label=outcome))
        successor = subtree_root.get("next")
        if successor is not None and (stack_root is not None or full_tree):
            dot, next_id = self._stack_to_dotgraph(dot, successor, stack_root, full_tree)
            dot.add_edge(pydot.Edge(node.get_name(), next_id))
        return dot, node.get_name()

    def to_dotgraph(self, full_tree: bool) -> pydot.Dot:
        with self._lock:
            if self._cached_dotgraph is not None and self._cached_full_tree == full_tree:
                return self._cached_dotgraph
            if self.tree is None or self.stack is None:
                dot = self._error_dotgraph()
            else:
                dot, _ = self._stack_to_dotgraph(pydot.Dot(graph_type="digraph"), self.tree, self.stack, full_tree)
            self._cached_dotgraph = dot
            self._cached_full_tree = full_tree
            return dot

    def _append_debug_data_to_item(self, parent_item: QStandardItem, debug_data: object) -> None:
        if isinstance(debug_data, (dict, list)):
            entries = debug_data.items() if isinstance(debug_data, dict) else enumerate(debug_data)
            for label, data in entries:
                child = QStandardItem(f"{label}: ")
                child.setEditable(False)
                child.setSelectable(False)
                self._append_debug_data_to_item(child, data)
                parent_item.appendRow(child)
        else:
            parent_item.setText(parent_item.text() + str(debug_data))

    def to_q_item_model(self) -> QStandardItemModel:
        with self._lock:
            if self._cached_item_model is not None:
                return self._cached_item_model
            model = QStandardItemModel()
            frame = self.stack
            while frame is not None:
                item = QStandardItem(frame["name"])
                item.setEditable(False)
                item.setSelectable(False)
                self._append_debug_data_to_item(item, frame["debug_data"])
                model.appendRow(item)
                frame = frame["next"]
                if frame is not None:
                    spacer = QStandardItem("──────────────────────────────")
                    spacer.setEditable(False)
                    spacer.setSelectable(False)
                    model.appendRow(spacer)
            self._cached_item_model = model
            return model

    def destroy(self) -> None:
        """Release both ROS subscriptions. Repeated calls are harmless."""
        with self._lock:
            subscriptions = self.tree_sub, self.stack_sub
            self.tree_sub = self.stack_sub = None
        for subscription in subscriptions:
            if subscription is not None:
                self._node.destroy_subscription(subscription)
