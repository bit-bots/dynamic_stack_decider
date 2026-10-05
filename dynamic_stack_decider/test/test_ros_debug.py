"""Exercise debug topics through a real ROS node and DDS publishers/subscribers."""

import json
import time
from collections.abc import Callable, Iterator

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from .runtime_elements import make_dsd


@pytest.fixture
def ros_node() -> Iterator[Node]:
    context = Context()
    rclpy.init(context=context)
    node = Node("dsd_debug_test", context=context)
    try:
        yield node
    finally:
        node.destroy_node()
        context.shutdown()


def wait_for(node: Node, predicate: Callable[[], bool]) -> None:
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        deadline = time.monotonic() + 8
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert predicate(), "ROS debug data was not received before the deadline"
    finally:
        executor.remove_node(node)
        executor.shutdown()


@pytest.mark.ros
def test_debug_topics_and_late_subscribers(ros_node: Node) -> None:
    dsd = make_dsd("-->\n$Chooser\n    GO --> @Idle\n", node=ros_node, debug_topic="test_debug")
    dsd.blackboard.results["Chooser"] = "GO"
    trees: list[String] = []
    stacks: list[String] = []
    actions: list[String] = []
    latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    ros_node.create_subscription(String, "test_debug/dsd_tree", trees.append, latched)
    ros_node.create_subscription(String, "test_debug/dsd_stack", stacks.append, 10)
    wait_for(ros_node, lambda: bool(trees) and dsd.debug_stack_publisher.get_subscription_count() > 0)
    dsd.update()
    wait_for(ros_node, lambda: bool(stacks))
    # A subscriber connecting after the update receives the active action too.
    ros_node.create_subscription(String, "test_debug/dsd_current_action", actions.append, latched)
    wait_for(ros_node, lambda: bool(actions))
    assert json.loads(trees[-1].data)["name"] == "Chooser"
    stack = json.loads(stacks[-1].data)
    assert stack["next"]["name"] == "Idle"
    assert actions[-1].data == "Idle"
