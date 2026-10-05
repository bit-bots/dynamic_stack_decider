"""Snapshot validation, rendering, and cache invalidation regressions."""

import copy
import importlib
import json
import os

import pytest

if os.environ.get("DSD_REQUIRE_GUI"):
    importlib.import_module("PyQt5.QtWidgets")
else:
    pytest.importorskip("PyQt5.QtWidgets")

from dynamic_stack_decider.tree import ActionTreeElement, DecisionTreeElement
from dynamic_stack_decider_visualization.dsd_follower import DsdFollower
from qt_dotgraph.pydotfactory import PydotFactory
from std_msgs.msg import String


def snapshots():
    root = DecisionTreeElement("Mode", None)
    first = ActionTreeElement("Wait", root)
    middle = ActionTreeElement("Work", root)
    last = ActionTreeElement("Wait", root)
    first.next_element = middle
    middle.next_element = last
    root.add_child_element(first, "GO")
    other = DecisionTreeElement("Other", root)
    other.add_child_element(ActionTreeElement("ElseAction", other), "ELSE")
    root.add_child_element(other, "ELSE")
    stack = {
        "id": id(root),
        "type": "decision",
        "name": "Mode",
        "debug_data": {},
        "activation_reason": None,
        "next": {
            "id": id(last),
            "type": "action",
            "name": "Wait",
            "debug_data": {"value": 1},
            "activation_reason": "GO",
            "next": None,
        },
    }
    return root.repr_dict(), stack


@pytest.fixture
def follower(node, qapp):
    receiver = DsdFollower(node, "/debug")
    yield receiver
    receiver.destroy()


def feed(follower, tree, stack):
    follower._tree_callback(String(data=json.dumps(tree)))
    follower._stack_callback(String(data=json.dumps(stack)))


def test_repeated_names_highlight_only_the_active_chain_element(follower):
    tree, stack = snapshots()
    feed(follower, tree, stack)
    graph = follower.to_dotgraph(False)
    active = [node.get_name() for node in graph.get_nodes() if node.get_color() == "orangered"]
    assert set(active) == {str(stack["id"]), str(stack["next"]["id"])}
    first_id = str(tree["children"]["GO"]["id"])
    assert graph.get_node(first_id)[0].get_color() == "gray"
    assert "digraph" in PydotFactory().create_dot(graph)


def test_full_tree_flag_is_part_of_cache_key(follower):
    tree, stack = snapshots()
    feed(follower, tree, stack)
    partial = follower.to_dotgraph(False)
    complete = follower.to_dotgraph(True)
    assert partial is not complete
    assert len(complete.get_nodes()) > len(partial.get_nodes())
    assert follower.to_dotgraph(True) is complete


def test_debug_changes_reuse_graph_but_refresh_item_model(follower):
    tree, stack = snapshots()
    feed(follower, tree, stack)
    graph = follower.to_dotgraph(False)
    model = follower.to_q_item_model()
    assert follower.to_q_item_model() is model
    stack["next"]["debug_data"]["value"] = 2
    follower._stack_callback(String(data=json.dumps(stack)))
    assert follower.to_dotgraph(False) is graph
    new_model = follower.to_q_item_model()
    assert new_model is not model
    assert new_model.item(2).child(0).text() == "value: 2"


def test_tree_reload_clears_old_stack_and_render_caches(follower):
    tree, stack = snapshots()
    feed(follower, tree, stack)
    old_graph = follower.to_dotgraph(False)
    replacement = copy.deepcopy(tree)
    replacement["id"] += 1
    follower._tree_callback(String(data=json.dumps(replacement)))
    assert follower.stack is None
    assert follower.to_dotgraph(False) is not old_graph
    follower._stack_callback(String(data=json.dumps(stack)))
    assert follower.stack is None
    stack["id"] = replacement["id"]
    follower._stack_callback(String(data=json.dumps(stack)))
    assert follower.stack is not None


def test_unchanged_tree_does_not_discard_stack(follower):
    tree, stack = snapshots()
    feed(follower, tree, stack)
    graph = follower.to_dotgraph(False)
    follower._tree_callback(String(data=json.dumps(tree)))
    assert follower.stack is not None
    assert follower.to_dotgraph(False) is graph


@pytest.mark.parametrize("payload", ["{", "null", "[]", '{"id": true}', '{"id":1,"name":"A","type":"invalid"}'])
def test_invalid_tree_messages_keep_last_valid_state(follower, node, payload):
    tree, stack = snapshots()
    feed(follower, tree, stack)
    follower._tree_callback(String(data=payload))
    assert follower.tree == tree
    assert follower.stack == stack
    node.logger.error.assert_called()


@pytest.mark.parametrize("payload", ["{", "[]", '{"id":1,"name":"A","type":"action","debug_data":{}}'])
def test_invalid_stack_messages_keep_last_valid_state(follower, node, payload):
    tree, stack = snapshots()
    feed(follower, tree, stack)
    follower._stack_callback(String(data=payload))
    assert follower.stack == stack
    node.logger.error.assert_called()


def test_inconsistent_stack_identity_is_rejected(follower, node):
    tree, stack = snapshots()
    feed(follower, tree, stack)
    invalid = copy.deepcopy(stack)
    invalid["next"]["name"] = "WrongName"
    follower._stack_callback(String(data=json.dumps(invalid)))
    assert follower.stack == stack
    node.logger.error.assert_called()


def test_nested_and_null_debug_values_are_shown(follower):
    tree, stack = snapshots()
    stack["next"]["debug_data"] = {"items": [None, {"enabled": False}]}
    feed(follower, tree, stack)
    debug = follower.to_q_item_model().item(2).child(0)
    assert debug.child(0).text() == "0: None"
    assert debug.child(1).child(0).text() == "enabled: False"


def test_destroy_releases_both_subscriptions_once(follower, node):
    follower.destroy()
    follower.destroy()
    assert len(node.destroyed) == 2
    assert set(node.destroyed) == {handle for handle, _, _ in node.subscriptions}


def test_ignored_keys_do_not_trigger_structural_changes():
    assert not DsdFollower.stack_dict_changed({"name": "A"}, {"name": "A", "debug_data": {}}, ["debug_data"])


def test_waiting_graph_is_cached(follower):
    assert follower.to_dotgraph(False) is follower.to_dotgraph(False)
