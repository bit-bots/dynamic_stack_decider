"""Execute persistent actions selected by reevaluable decisions."""

from __future__ import annotations

import copy
import importlib
import inspect
import json
import pkgutil
import sys
import traceback
from pathlib import Path
from typing import Generic

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from dynamic_stack_decider.abstract_stack_element import AbstractStackElement
from dynamic_stack_decider.logger import get_logger
from dynamic_stack_decider.parser import DsdParser
from dynamic_stack_decider.tree import AbstractTreeElement, ActionTreeElement, DecisionTreeElement, Tree
from dynamic_stack_decider.types import BlackboardT, StackData


def discover_elements(path: str | Path) -> dict[str, type[AbstractStackElement[BlackboardT]]]:
    """Discover locally defined element classes, logging modules that cannot be imported."""
    path = Path(path)
    if not path.is_file() and not path.is_dir():
        raise ValueError(f"Path {path} is not a directory or file")
    elements: dict[str, type[AbstractStackElement[BlackboardT]]] = {}
    original_pythonpath = sys.path.copy()

    def discover_module(module_name: str) -> None:
        try:
            if module_name in sys.modules:
                importlib.reload(sys.modules[module_name])
            module = importlib.import_module(module_name)
            elements.update(
                inspect.getmembers(
                    module,
                    lambda member: (
                        inspect.isclass(member)
                        and inspect.getmodule(member) == module
                        and issubclass(member, AbstractStackElement)
                        and not inspect.isabstract(member)
                    ),
                )
            )
        except Exception as error:
            get_logger().error(f"Cannot import element module {module_name}: {error}")

    try:
        if path.is_file():
            is_package = path.name == "__init__.py"
            sys.path.append(str(path.parent.parent if is_package else path.parent))
            discover_module(path.parent.name if is_package else path.stem)
        else:
            sys.path.append(str(path.parent))
            discover_module(path.name)
            for _, module_name, _ in pkgutil.walk_packages([str(path)], prefix=f"{path.name}."):
                discover_module(module_name)
    finally:
        sys.path[:] = original_pythonpath
    return elements


class DSD(Generic[BlackboardT]):
    """Retain an active decision path and execute one action per update.

    Decisions descend immediately to their selected child. Actions remain active
    until they pop or a reevaluated decision selects a different branch. Completing
    an action in a chain initializes its successor for the next update.
    """

    def __init__(self, blackboard: BlackboardT, debug_topic: str | None = None, node: Node | None = None) -> None:
        self.blackboard = blackboard
        self.node = node
        self.tree: Tree[BlackboardT] | None = None
        self.start_element: AbstractTreeElement[BlackboardT] | None = None
        self.stack: list[tuple[AbstractTreeElement[BlackboardT], AbstractStackElement[BlackboardT]]] = []
        self.actions: dict[str, type[AbstractActionElement[BlackboardT]]] = {}
        self.decisions: dict[str, type[AbstractDecisionElement[BlackboardT]]] = {}
        self.stack_exec_index = -1
        self.stack_reevaluate = False
        self.do_not_reevaluate = False
        self._executing = False
        self._stack_revision = 0
        self.last_error: Exception | None = None
        self.debug_active = debug_topic is not None and node is not None
        self.debug_active_action_cache: str | None = None
        self.debug_tree_publisher: Publisher[String] | None = None
        self.debug_stack_publisher: Publisher[String] | None = None
        self.debug_current_action_publisher: Publisher[String] | None = None
        if self.debug_active and node is not None:
            latched_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.debug_tree_publisher = node.create_publisher(String, f"{debug_topic}/dsd_tree", latched_qos)
            self.debug_stack_publisher = node.create_publisher(String, f"{debug_topic}/dsd_stack", 10)
            self.debug_current_action_publisher = node.create_publisher(
                String, f"{debug_topic}/dsd_current_action", latched_qos
            )

    def register_actions(self, module_path: str | Path) -> None:
        """Replace the action registry with classes discovered at a file or package path."""
        discovered: dict[str, type[AbstractStackElement[BlackboardT]]] = discover_elements(module_path)
        self.actions = {name: cls for name, cls in discovered.items() if issubclass(cls, AbstractActionElement)}

    def register_decisions(self, module_path: str | Path) -> None:
        """Replace the decision registry with classes discovered at a file or package path."""
        discovered: dict[str, type[AbstractStackElement[BlackboardT]]] = discover_elements(module_path)
        self.decisions = {name: cls for name, cls in discovered.items() if issubclass(cls, AbstractDecisionElement)}

    def load_behavior(self, path: str | Path) -> None:
        """Parse and bind a behavior before canceling the previously loaded behavior."""
        parser: DsdParser[BlackboardT] = DsdParser(self.node)
        tree = parser.parse(path)
        if tree.root_element is None:
            raise ValueError("Behavior has no root element")
        self._bind_modules(tree.root_element)
        self.set_start_element(tree.root_element)
        self.tree = tree
        self.debug_active_action_cache = None
        try:
            self.debug_publish_tree()
        except Exception as error:
            self._record_error(error)

    def _bind_modules(self, element: AbstractTreeElement[BlackboardT]) -> None:
        pending = [element]
        while pending:
            current = pending.pop()
            if isinstance(current, ActionTreeElement):
                if current.name not in self.actions:
                    raise ValueError(
                        f"Action {current.name!r} is not registered ({current.source_file}:{current.source_line})"
                    )
                current.module = self.actions[current.name]
                if current.next_element is not None:
                    pending.append(current.next_element)
            elif isinstance(current, DecisionTreeElement):
                if current.name not in self.decisions:
                    raise ValueError(
                        f"Decision {current.name!r} is not registered ({current.source_file}:{current.source_line})"
                    )
                current.module = self.decisions[current.name]
                pending.extend(current.children.values())
            else:
                raise ValueError(f"Unknown tree element type {type(current).__name__}")

    def _init_element(self, element: AbstractTreeElement[BlackboardT]) -> AbstractStackElement[BlackboardT]:
        if element.module is None:
            raise ValueError(f"Element {element.name!r} has no bound implementation")
        return element.module(self.blackboard, self, copy.deepcopy(element.parameters))

    def _append(self, element: AbstractTreeElement[BlackboardT]) -> None:
        revision = self._stack_revision
        instance = self._init_element(element)
        if self._stack_revision != revision:
            raise RuntimeError("Element constructor changed the stack")
        self.stack.append((element, instance))
        self._stack_revision += 1

    def _record_error(self, error: Exception) -> None:
        if self.last_error is None:
            self.last_error = error
        get_logger().error("".join(traceback.format_exception(type(error), error, error.__traceback__)))

    def _discard_from(self, index: int) -> bool:
        """Detach frames before cleanup; return whether cleanup left the remaining stack intact."""
        removed = self.stack[index:]
        del self.stack[index:]
        self._stack_revision += 1
        revision = self._stack_revision
        for _, instance in reversed(removed):
            try:
                instance.on_pop()
            except Exception as error:
                self._record_error(error)
        return self._stack_revision == revision

    def set_start_element(self, start_element: AbstractTreeElement[BlackboardT]) -> None:
        """Cancel the active path and initialize a new start element without executing it."""
        self.stack_reevaluate = False
        self.do_not_reevaluate = False
        if not self._discard_from(0):
            raise RuntimeError("Cleanup changed the stack while replacing the behavior")
        self.start_element = start_element
        self._append(start_element)

    def interrupt(self) -> None:
        """Cancel every active frame and initialize the declared start element for the next update."""
        if self.start_element is None:
            raise RuntimeError("No behavior has been loaded")
        self.stack_reevaluate = False
        self.do_not_reevaluate = False
        if self._discard_from(0):
            self._append(self.start_element)

    def _reevaluate_stack(self) -> bool:
        """Return true when a callback canceled the remainder of this update."""
        self.stack_reevaluate = True
        revision = self._stack_revision
        try:
            for index, (tree_element, instance) in enumerate(self.stack[:-1]):
                self.stack_exec_index = index
                if not isinstance(instance, AbstractDecisionElement):
                    continue
                reevaluate = instance.get_reevaluate()
                if not self.stack_reevaluate or self._stack_revision != revision:
                    return True
                if not reevaluate:
                    continue
                if not isinstance(tree_element, DecisionTreeElement):
                    raise TypeError("Decision instance is not bound to a decision tree element")
                result = instance.perform(True)
                if not self.stack_reevaluate or self._stack_revision != revision:
                    return True
                child = tree_element.get_child(result)
                if child.activation_reason != self.stack[index + 1][0].activation_reason:
                    self.stack_reevaluate = False
                    if not self._discard_from(index + 1):
                        return True
                    self._append(child)
                    return False
            return False
        finally:
            self.stack_reevaluate = False
            self.stack_exec_index = -1

    def _execute_top(self) -> None:
        while self.stack:
            tree_element, instance = self.stack[-1]
            if isinstance(instance, AbstractActionElement) and instance.never_reevaluate:
                self.set_do_not_reevaluate()
            revision = self._stack_revision
            result = instance.perform()
            if self._stack_revision != revision:
                return
            if not isinstance(instance, AbstractDecisionElement):
                return
            if not isinstance(tree_element, DecisionTreeElement):
                raise TypeError("Decision instance is not bound to a decision tree element")
            self._append(tree_element.get_child(result))

    def update(self, reevaluate: bool = True) -> None:
        """Reevaluate the active path and run its action, logging failures in ``last_error``.

        Do not call update recursively or concurrently. Callbacks that change the
        stack cancel the rest of the current traversal. Debugging runs after the
        tick and cannot prevent behavior execution.
        """
        if self._executing:
            raise RuntimeError("DSD.update() is already running")
        self.last_error = None
        self._executing = True
        try:
            if not self.stack:
                raise RuntimeError("No active behavior; load a behavior before update()")
            if reevaluate and not self.do_not_reevaluate and self._reevaluate_stack():
                return
            if reevaluate:
                self.do_not_reevaluate = False
            self._execute_top()
        except Exception as error:
            self._record_error(error)
        finally:
            self.stack_reevaluate = False
            self.stack_exec_index = -1
            self._executing = False
            for publish in (self.debug_publish_stack, self.debug_publish_current_action):
                try:
                    publish()
                except Exception as error:
                    self._record_error(error)

    def push(self, element: AbstractTreeElement[BlackboardT]) -> None:
        """Append a frame and execute it immediately when called outside an update."""
        self._append(element)
        if not self._executing:
            self.update(False)

    def pop(self) -> None:
        """Complete the current frame, initializing a chain successor for the next update.

        During reevaluation, remove the reevaluated frame and everything above it.
        Popping the only frame remains a no-op. Suppression is retained when
        advancing a chain, matching the existing sequence contract.
        """
        if len(self.stack) <= 1:
            return
        if self.stack_reevaluate:
            self.stack_reevaluate = False
            if self.stack_exec_index > 0:
                self._discard_from(self.stack_exec_index)
            self.do_not_reevaluate = False
            return
        tree_element = self.stack[-1][0]
        successor = tree_element.next_element if isinstance(tree_element, ActionTreeElement) else None
        if not self._discard_from(len(self.stack) - 1):
            return
        if successor is not None:
            self._append(successor)
        else:
            self.do_not_reevaluate = False

    def set_do_not_reevaluate(self) -> None:
        """Suppress reevaluation on the next update."""
        self.do_not_reevaluate = True

    def get_stack(self) -> list[tuple[AbstractTreeElement[BlackboardT], AbstractStackElement[BlackboardT]]]:
        """Return the active path, ordered from root to current frame. Treat it as read-only."""
        return self.stack

    def debug_publish_stack(self) -> None:
        publisher = self.debug_stack_publisher
        if not self.debug_active or publisher is None or publisher.get_subscription_count() == 0:
            return
        data: StackData | None = None
        for tree_element, instance in reversed(self.stack):
            data = {
                **instance.repr_dict(),
                "activation_reason": tree_element.activation_reason,
                "id": id(tree_element),
                "next": data,
            }
        publisher.publish(String(data=json.dumps(data, default=str)))

    def debug_publish_tree(self) -> None:
        if self.debug_active and self.debug_tree_publisher is not None and self.tree is not None:
            self.debug_tree_publisher.publish(String(data=json.dumps(self.tree.repr_dict(), default=str)))

    def debug_publish_current_action(self) -> None:
        publisher = self.debug_current_action_publisher
        if not self.debug_active or publisher is None:
            return
        top = self.stack[-1][1] if self.stack else None
        name = top.name if isinstance(top, AbstractActionElement) else ""
        if name != self.debug_active_action_cache:
            publisher.publish(String(data=name))
            self.debug_active_action_cache = name
