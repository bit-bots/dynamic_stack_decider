"""
Runtime behavior tests for the DSD execution engine (``dsd.py``).

These tests drive a DSD through ``update()`` and assert on the resulting stack and execution order.
They aim to cover every runtime code path: pushing/popping, sequences (chains), reevaluation
(including ELSE handling), the ``never_reevaluate`` / ``do_not_reevaluate`` flags, interrupts,
decisions and subtrees following actions, error handling and the debug publishing.
"""

import json

import pytest
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement

from .runtime_elements import Blackboard, make_dsd, stack_names


class FakePublisher:
    """Records published messages and reports a configurable subscription count."""

    def __init__(self):
        self.messages = []
        self.subscription_count = 1

    def publish(self, msg):
        self.messages.append(msg)

    def get_subscription_count(self):
        return self.subscription_count


class FakeNode:
    """A stand-in for an rclpy node that just hands out FakePublishers."""

    def __init__(self):
        self.publishers = {}

    def create_publisher(self, msg_type, topic, qos):
        publisher = FakePublisher()
        self.publishers[topic] = publisher
        return publisher


# ---------------------------------------------------------------------------
# Basic decision / action flow
# ---------------------------------------------------------------------------
def test_start_element_on_stack_after_load():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """
    )
    # After loading, only the start element is on the stack and nothing has been performed yet
    assert stack_names(dsd) == ["Chooser"]
    assert dsd.blackboard.events == []


def test_decision_pushes_action():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    dsd.update()

    assert stack_names(dsd) == ["Chooser", "Idle"]
    assert dsd.blackboard.events == [("decide", "Chooser", False), ("perform", "Idle")]


def test_persistent_action_stays_on_stack():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    for _ in range(3):
        dsd.update()

    assert stack_names(dsd) == ["Chooser", "Idle"]
    # The action was performed on every update
    assert dsd.blackboard.events.count(("perform", "Idle")) == 3


def test_action_pops_and_decision_redecides():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @StepA
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    dsd.update()
    # StepA popped itself again, so only the decision remains
    assert stack_names(dsd) == ["Chooser"]
    assert ("pop", "StepA") in dsd.blackboard.events

    dsd.update()
    # The decision pushes and runs StepA again
    assert dsd.blackboard.events.count(("perform", "StepA")) == 2


def test_nested_decisions():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            DOWN --> $Guard
                GO --> @Idle
        """
    )
    dsd.blackboard.results["Chooser"] = "DOWN"
    dsd.blackboard.results["Guard"] = "GO"

    dsd.update()

    assert stack_names(dsd) == ["Chooser", "Guard", "Idle"]


# ---------------------------------------------------------------------------
# Sequences (chains via next_element)
# ---------------------------------------------------------------------------
def test_sequence_advances_one_action_per_tick():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @StepA, @StepB, @StepC
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    dsd.update()
    # StepA performed and popped, advancing to StepB (not yet performed)
    assert stack_names(dsd) == ["Chooser", "StepB"]

    dsd.update()
    assert stack_names(dsd) == ["Chooser", "StepC"]

    dsd.update()
    # The last action has no successor, so it pops normally and the decision remains
    assert stack_names(dsd) == ["Chooser"]

    # Every action was performed and popped exactly once, in order
    assert [e for e in dsd.blackboard.events if e[0] == "pop"] == [
        ("pop", "StepA"),
        ("pop", "StepB"),
        ("pop", "StepC"),
    ]


def test_sequence_with_multi_tick_action():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @CountingAction + count:2, @Idle
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    dsd.update()
    # CountingAction needs two performs before it advances
    assert stack_names(dsd) == ["Chooser", "CountingAction"]

    dsd.update()
    # After the second perform it popped and advanced to Idle
    assert stack_names(dsd) == ["Chooser", "Idle"]
    assert dsd.blackboard.events.count(("perform", "CountingAction")) == 2


def test_decision_after_action():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @StepA, $Guard
                ON --> @Idle
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"
    dsd.blackboard.results["Guard"] = "ON"

    dsd.update()
    # StepA ran and advanced to the decision Guard
    assert stack_names(dsd) == ["Chooser", "Guard"]

    dsd.update()
    # Guard now decides and pushes its child
    assert stack_names(dsd) == ["Chooser", "Guard", "Idle"]


def test_subtree_after_action():
    dsd = make_dsd(
        """
        #Sub
        $Guard
            ON --> @Idle
        -->T
        $Chooser
            GO --> @StepA, #Sub
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"
    dsd.blackboard.results["Guard"] = "ON"

    dsd.update()
    assert stack_names(dsd) == ["Chooser", "Guard"]

    dsd.update()
    assert stack_names(dsd) == ["Chooser", "Guard", "Idle"]


# ---------------------------------------------------------------------------
# Reevaluation
# ---------------------------------------------------------------------------
def test_reevaluation_rebuilds_stack_on_change():
    dsd = make_dsd(
        """
        -->T
        $Guard
            A --> @Idle
            B --> @Run
        """
    )
    dsd.blackboard.results["Guard"] = "A"

    dsd.update()
    assert stack_names(dsd) == ["Guard", "Idle"]

    # Same result -> no rebuild
    dsd.update()
    assert stack_names(dsd) == ["Guard", "Idle"]

    # Changed result -> the stack above the decision is dropped and rebuilt
    dsd.blackboard.results["Guard"] = "B"
    dsd.update()
    assert stack_names(dsd) == ["Guard", "Run"]
    assert ("pop", "Idle") in dsd.blackboard.events


def test_reevaluation_reruns_only_reevaluating_decisions():
    # Guard reevaluates, the nested Chooser does not
    dsd = make_dsd(
        """
        -->T
        $Guard
            GO --> $Chooser
                A --> @Idle
                B --> @Run
        """
    )
    dsd.blackboard.results["Guard"] = "GO"
    dsd.blackboard.results["Chooser"] = "A"

    dsd.update()
    assert stack_names(dsd) == ["Guard", "Chooser", "Idle"]

    # Changing the non-reevaluating Chooser has no effect while it is in the middle of the stack
    dsd.blackboard.results["Chooser"] = "B"
    dsd.update()
    assert stack_names(dsd) == ["Guard", "Chooser", "Idle"]


def test_reevaluation_else_branch():
    dsd = make_dsd(
        """
        -->T
        $ElseGuard
            A --> @Idle
            ELSE --> @Run
        """
    )
    dsd.blackboard.results["ElseGuard"] = "A"

    dsd.update()
    assert stack_names(dsd) == ["ElseGuard", "Idle"]

    # An unknown result maps to the ELSE branch -> rebuild into the ELSE child
    dsd.blackboard.results["ElseGuard"] = "SOMETHING_ELSE"
    dsd.update()
    assert stack_names(dsd) == ["ElseGuard", "Run"]

    # Still an ELSE result -> no rebuild (the activation reason stays "ELSE")
    dsd.blackboard.results["ElseGuard"] = "YET_ANOTHER"
    dsd.update()
    assert stack_names(dsd) == ["ElseGuard", "Run"]
    assert dsd.blackboard.events.count(("pop", "Run")) == 0

    # Back to a concrete result -> rebuild into the concrete child
    dsd.blackboard.results["ElseGuard"] = "A"
    dsd.update()
    assert stack_names(dsd) == ["ElseGuard", "Idle"]


# ---------------------------------------------------------------------------
# never_reevaluate / do_not_reevaluate
# ---------------------------------------------------------------------------
def test_never_reevaluate_parameter_suppresses_reevaluation():
    dsd = make_dsd(
        """
        -->T
        $Guard
            A --> @Idle + r:false
            B --> @Run
        """
    )
    dsd.blackboard.results["Guard"] = "A"

    dsd.update()
    assert stack_names(dsd) == ["Guard", "Idle"]

    # Because Idle has never_reevaluate set, the guard is not reevaluated and the change is ignored
    dsd.blackboard.results["Guard"] = "B"
    for _ in range(3):
        dsd.update()
    assert stack_names(dsd) == ["Guard", "Idle"]


def test_do_not_reevaluate_delays_reevaluation_by_one_tick():
    dsd = make_dsd(
        """
        -->T
        $Guard
            A --> @DoNotReevalOnce
            B --> @Run
        """
    )
    dsd.blackboard.results["Guard"] = "A"

    dsd.update()
    assert stack_names(dsd) == ["Guard", "DoNotReevalOnce"]

    # The action requested do_not_reevaluate once, so this update skips reevaluation
    dsd.blackboard.results["Guard"] = "B"
    dsd.update()
    assert stack_names(dsd) == ["Guard", "DoNotReevalOnce"]

    # The flag was reset, so the next update reevaluates and rebuilds
    dsd.update()
    assert stack_names(dsd) == ["Guard", "Run"]


def test_do_not_reevaluate_reset_on_normal_pop():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @DoNotReevalThenPop
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    dsd.update()
    # The action set do_not_reevaluate but then popped (no successor), so the flag is reset
    assert dsd.do_not_reevaluate is False
    assert stack_names(dsd) == ["Chooser"]


def test_do_not_reevaluate_preserved_across_sequence_advance():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @DoNotReevalThenPop, @Idle
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    dsd.update()
    # The action advanced to the next chain element and kept do_not_reevaluate set
    assert stack_names(dsd) == ["Chooser", "Idle"]
    assert dsd.do_not_reevaluate is True


# ---------------------------------------------------------------------------
# Interrupts
# ---------------------------------------------------------------------------
def test_external_interrupt_resets_stack():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    dsd.update()
    assert stack_names(dsd) == ["Chooser", "Idle"]
    top_before = dsd.stack[0][1]

    dsd.interrupt()
    assert stack_names(dsd) == ["Chooser"]
    # A fresh start element instance is created
    assert dsd.stack[0][1] is not top_before


def test_interrupt_from_within_an_action():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Interrupter
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    dsd.update()
    # The action interrupted the DSD, clearing the stack back to the start element
    assert stack_names(dsd) == ["Chooser"]


def test_interrupt_during_reevaluation():
    dsd = make_dsd(
        """
        -->T
        $InterruptOnReeval
            GO --> @Idle
        """
    )
    dsd.blackboard.results["InterruptOnReeval"] = "GO"

    dsd.update()
    assert stack_names(dsd) == ["InterruptOnReeval", "Idle"]

    # On the next update the root decision is reevaluated and interrupts, resetting the stack
    dsd.update()
    assert stack_names(dsd) == ["InterruptOnReeval"]


# ---------------------------------------------------------------------------
# pop edge cases
# ---------------------------------------------------------------------------
def test_pop_of_only_element_is_noop():
    # A behavior whose root is a self-popping action
    dsd = make_dsd(
        """
        -->T
        @StepA
        """
    )

    dsd.update()
    # The action tried to pop itself, but it is the only element, so the stack is unchanged
    assert stack_names(dsd) == ["StepA"]
    assert ("pop", "StepA") not in dsd.blackboard.events


def test_reevaluation_pop_at_root_does_not_shorten_stack():
    # A reevaluating root decision that pops itself during reevaluation. Because it is at the bottom
    # of the stack, no element above the reevaluation index is dropped and the update returns early.
    dsd = make_dsd(
        """
        -->T
        $PopOnReeval
            GO --> @Idle
        """
    )
    dsd.blackboard.results["PopOnReeval"] = "GO"

    dsd.update()
    assert stack_names(dsd) == ["PopOnReeval", "Idle"]

    performed_before = dsd.blackboard.events.count(("perform", "Idle"))
    dsd.update()
    # The stack is untouched and, because reevaluation stopped, the top action was not performed again
    assert stack_names(dsd) == ["PopOnReeval", "Idle"]
    assert dsd.blackboard.events.count(("perform", "Idle")) == performed_before
    assert dsd.stack_reevaluate is False


def test_reevaluation_pop_in_the_middle_shortens_stack():
    # A popped decision cancels reevaluation before the removed frame can be inspected.
    dsd = make_dsd(
        """
        -->T
        $Guard
            GO --> $PopOnReeval
                X --> @Idle
        """
    )
    dsd.blackboard.results["Guard"] = "GO"
    dsd.blackboard.results["PopOnReeval"] = "X"

    dsd.update()
    assert stack_names(dsd) == ["Guard", "PopOnReeval", "Idle"]

    dsd.update()
    # PopOnReeval and its child were dropped during reevaluation
    assert stack_names(dsd) == ["Guard"]
    assert ("pop", "Idle") in dsd.blackboard.events


# ---------------------------------------------------------------------------
# Error handling
# ---------------------------------------------------------------------------
def test_exception_in_perform_is_swallowed():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Raiser
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"

    # The exception raised inside the action must not propagate out of update()
    dsd.update()
    assert stack_names(dsd) == ["Chooser", "Raiser"]
    assert ("perform", "Raiser") in dsd.blackboard.events


def test_unknown_decision_result_is_swallowed():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """
    )
    dsd.blackboard.results["Chooser"] = "UNKNOWN"

    # get_child raises a KeyError which is caught inside update()
    dsd.update()
    assert stack_names(dsd) == ["Chooser"]


def test_decision_returning_none_is_reported():
    dsd = make_dsd(
        """
        -->T
        $ReturnNone
            GO --> @Idle
        """
    )

    dsd.update()
    assert isinstance(dsd.last_error, ValueError)
    assert stack_names(dsd) == ["ReturnNone"]


# ---------------------------------------------------------------------------
# _bind_modules error paths
# ---------------------------------------------------------------------------
def test_unknown_action_name_raises_on_load():
    with pytest.raises(ValueError):
        make_dsd(
            """
            -->T
            $Chooser
                GO --> @DoesNotExist
            """
        )


def test_unknown_decision_name_raises_on_load():
    with pytest.raises(ValueError):
        make_dsd(
            """
            -->T
            $DoesNotExist
                GO --> @Idle
            """
        )


def test_bind_modules_rejects_unknown_element_type():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """
    )

    class Bogus:
        pass

    with pytest.raises(ValueError):
        dsd._bind_modules(Bogus())


# ---------------------------------------------------------------------------
# Misc DSD API
# ---------------------------------------------------------------------------
def test_get_stack_returns_the_stack():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """
    )
    assert dsd.get_stack() is dsd.stack


def test_set_do_not_reevaluate():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """
    )
    assert dsd.do_not_reevaluate is False
    dsd.set_do_not_reevaluate()
    assert dsd.do_not_reevaluate is True


def test_stack_holds_bound_instances():
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """
    )
    dsd.blackboard.results["Chooser"] = "GO"
    dsd.update()

    root_tree, root_instance = dsd.stack[0]
    assert isinstance(root_instance, AbstractDecisionElement)
    top_tree, top_instance = dsd.stack[-1]
    assert isinstance(top_instance, AbstractActionElement)
    # The instances share the blackboard
    assert root_instance.blackboard is dsd.blackboard is top_instance.blackboard


# ---------------------------------------------------------------------------
# Debug publishing
# ---------------------------------------------------------------------------
def test_debug_tree_published_on_load():
    node = FakeNode()
    make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """,
        node=node,
        debug_topic="dbg",
    )

    tree_messages = node.publishers["dbg/dsd_tree"].messages
    assert len(tree_messages) == 1
    tree = json.loads(tree_messages[0].data)
    assert tree["type"] == "decision"
    assert tree["name"] == "Chooser"
    assert "id" in tree
    assert tree["children"]["GO"]["type"] == "action"


def test_debug_stack_and_current_action_published():
    node = FakeNode()
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """,
        node=node,
        debug_topic="dbg",
    )
    dsd.blackboard.results["Chooser"] = "GO"

    dsd.update()

    stack_messages = node.publishers["dbg/dsd_stack"].messages
    assert stack_messages
    stack = json.loads(stack_messages[-1].data)
    # Bottom of the stack is the decision, above it the action, matched via the stable id
    assert stack["type"] == "decision" and "id" in stack
    assert stack["next"]["type"] == "action"
    assert stack["next"]["name"] == "Idle"
    assert stack["next"]["next"] is None

    # The current action is published exactly once (it does not change while Idle stays on top)
    action_messages = node.publishers["dbg/dsd_current_action"].messages
    dsd.update()
    dsd.update()
    assert [m.data for m in action_messages] == ["Idle"]


def test_debug_stack_not_published_without_subscribers():
    node = FakeNode()
    dsd = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """,
        node=node,
        debug_topic="dbg",
    )
    dsd.blackboard.results["Chooser"] = "GO"
    node.publishers["dbg/dsd_stack"].subscription_count = 0

    dsd.update()

    assert node.publishers["dbg/dsd_stack"].messages == []


def test_fresh_blackboard_between_dsds():
    bb1 = Blackboard()
    bb2 = Blackboard()
    dsd1 = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Idle
        """,
        blackboard=bb1,
    )
    dsd2 = make_dsd(
        """
        -->T
        $Chooser
            GO --> @Run
        """,
        blackboard=bb2,
    )
    dsd1.blackboard.results["Chooser"] = "GO"
    dsd2.blackboard.results["Chooser"] = "GO"

    dsd1.update()
    assert ("perform", "Idle") in bb1.events
    assert bb2.events == []


def test_interrupt_cleans_every_frame_once_in_reverse_order(monkeypatch):
    dsd = make_dsd("-->\n$Chooser\n    GO --> $Guard\n        GO --> @Idle\n")
    dsd.blackboard.results.update(Chooser="GO", Guard="GO")
    dsd.update()
    cleaned = []
    for _, instance in dsd.stack:
        monkeypatch.setattr(instance, "on_pop", lambda name=instance.name: cleaned.append(name))
    dsd.interrupt()
    assert cleaned == ["Idle", "Guard", "Chooser"]
    assert stack_names(dsd) == ["Chooser"]
    assert dsd.last_error is None


def test_interrupt_clears_reevaluation_suppression():
    dsd = make_dsd("-->\n$Guard\n    A --> @Idle + r:false\n    B --> @Run\n")
    dsd.blackboard.results["Guard"] = "A"
    dsd.update()
    assert dsd.do_not_reevaluate
    dsd.interrupt()
    assert not dsd.do_not_reevaluate
    dsd.blackboard.results["Guard"] = "B"
    dsd.update()
    assert stack_names(dsd) == ["Guard", "Run"]


@pytest.mark.parametrize("decision", ["InterruptOnReeval", "PopOnReeval"])
def test_stack_mutation_during_reevaluation_does_not_log_an_internal_error(decision):
    dsd = make_dsd(f"-->\n$Guard\n    GO --> ${decision}\n        GO --> @Idle\n")
    dsd.blackboard.results.update(Guard="GO", **{decision: "GO"})
    dsd.update()
    dsd.update()
    assert stack_names(dsd) == ["Guard"]
    assert dsd.last_error is None
    assert not dsd.stack_reevaluate


def test_interrupt_during_normal_decision_does_not_push_stale_outcome(monkeypatch):
    dsd = make_dsd("-->\n$Chooser\n    GO --> @Idle\n")
    root = dsd.stack[0][1]

    def perform(reevaluate=False):
        dsd.interrupt()
        return "GO"

    monkeypatch.setattr(root, "perform", perform)
    dsd.update()
    assert stack_names(dsd) == ["Chooser"]
    assert dsd.blackboard.events == []
    assert dsd.last_error is None


def test_interrupt_from_reevaluation_predicate_cancels_tick(monkeypatch):
    dsd = make_dsd("-->\n$Guard\n    GO --> @Idle\n")
    dsd.blackboard.results["Guard"] = "GO"
    dsd.update()

    def get_reevaluate():
        dsd.interrupt()
        return True

    monkeypatch.setattr(dsd.stack[0][1], "get_reevaluate", get_reevaluate)
    dsd.update()
    assert stack_names(dsd) == ["Guard"]
    assert dsd.last_error is None


def test_reevaluation_error_resets_traversal_state(monkeypatch):
    dsd = make_dsd("-->\n$Guard\n    GO --> @Idle\n")
    dsd.blackboard.results["Guard"] = "GO"
    dsd.update()

    def fail(reevaluate=False):
        raise ValueError("broken guard")

    monkeypatch.setattr(dsd.stack[0][1], "perform", fail)
    dsd.update()
    assert isinstance(dsd.last_error, ValueError)
    assert not dsd.stack_reevaluate
    assert dsd.stack_exec_index == -1
    monkeypatch.setattr(dsd.stack[0][1], "get_reevaluate", lambda: False)
    dsd.update()
    assert dsd.last_error is None
    assert dsd.blackboard.events.count(("perform", "Idle")) == 2


def test_cleanup_exception_does_not_skip_other_frames(monkeypatch):
    dsd = make_dsd("-->\n$Guard\n    A --> $Chooser\n        GO --> @Idle\n    B --> @Run\n")
    dsd.blackboard.results.update(Guard="A", Chooser="GO")
    dsd.update()
    cleaned = []

    def fail():
        cleaned.append("Idle")
        raise ValueError("cleanup failed")

    monkeypatch.setattr(dsd.stack[-1][1], "on_pop", fail)
    monkeypatch.setattr(dsd.stack[-2][1], "on_pop", lambda: cleaned.append("Chooser"))
    dsd.blackboard.results["Guard"] = "B"
    dsd.update()
    assert cleaned == ["Idle", "Chooser"]
    assert stack_names(dsd) == ["Guard", "Run"]
    assert isinstance(dsd.last_error, ValueError)


def test_cleanup_interrupt_does_not_push_stale_branch(monkeypatch):
    dsd = make_dsd("-->\n$Guard\n    A --> @Idle\n    B --> @Run\n")
    dsd.blackboard.results["Guard"] = "A"
    dsd.update()
    monkeypatch.setattr(dsd.stack[-1][1], "on_pop", dsd.interrupt)
    dsd.blackboard.results["Guard"] = "B"
    dsd.update()
    assert stack_names(dsd) == ["Guard"]
    assert ("perform", "Run") not in dsd.blackboard.events
    assert dsd.last_error is None


def test_invalid_reevaluation_outcome_keeps_existing_action():
    dsd = make_dsd("-->\n$Guard\n    GO --> @Idle\n")
    dsd.blackboard.results["Guard"] = "GO"
    dsd.update()
    dsd.blackboard.results["Guard"] = "INVALID"
    dsd.update()
    assert stack_names(dsd) == ["Guard", "Idle"]
    assert ("pop", "Idle") not in dsd.blackboard.events
    assert isinstance(dsd.last_error, KeyError)


def test_chain_cleanup_precedes_successor_construction(monkeypatch):
    dsd = make_dsd("-->\n$Chooser\n    GO --> @Idle, @Run\n")
    dsd.blackboard.results["Chooser"] = "GO"
    dsd.update()
    order = []
    monkeypatch.setattr(dsd.stack[-1][1], "on_pop", lambda: order.append("cleanup"))
    original_init = dsd._init_element

    def initialize(element):
        order.append("initialize")
        return original_init(element)

    monkeypatch.setattr(dsd, "_init_element", initialize)
    dsd.pop()
    assert order == ["cleanup", "initialize"]
    assert stack_names(dsd) == ["Chooser", "Run"]
    assert ("perform", "Run") not in dsd.blackboard.events


def test_parameters_are_independent_of_tree_and_next_activation():
    dsd = make_dsd("-->\n$Chooser\n    GO --> @Idle + items:[1]\n")
    dsd.blackboard.results["Chooser"] = "GO"
    dsd.update()
    dsd.stack[-1][1].parameters["items"].append(2)
    dsd.pop()
    dsd.update()
    assert dsd.stack[-1][1].parameters == {"items": [1]}
    assert dsd.stack[-1][0].parameters == {"items": [1]}


def test_failed_load_keeps_current_behavior(tmp_path):
    dsd = make_dsd("-->\n$Chooser\n    GO --> @Idle\n")
    dsd.blackboard.results["Chooser"] = "GO"
    dsd.update()
    old_tree = dsd.tree
    old_stack = list(dsd.stack)
    bad = tmp_path / "unknown.dsd"
    bad.write_text("-->\n@Unregistered\n")
    with pytest.raises(ValueError, match="not registered"):
        dsd.load_behavior(bad)
    assert dsd.tree is old_tree
    assert dsd.stack == old_stack
    assert ("pop", "Idle") not in dsd.blackboard.events


def test_behavior_reload_cleans_old_action(tmp_path):
    dsd = make_dsd("-->\n$Chooser\n    GO --> @Idle\n")
    dsd.blackboard.results["Chooser"] = "GO"
    dsd.update()
    path = tmp_path / "replacement.dsd"
    path.write_text("-->\n@Run\n")
    dsd.load_behavior(path)
    assert stack_names(dsd) == ["Run"]
    assert dsd.blackboard.events[-1] == ("pop", "Idle")
    assert ("perform", "Run") not in dsd.blackboard.events


def test_deep_decision_path_does_not_use_python_recursion():
    from dynamic_stack_decider.dsd import DSD
    from dynamic_stack_decider.tree import ActionTreeElement, DecisionTreeElement

    from .runtime_elements import Guard, Idle

    root = DecisionTreeElement("Guard", None)
    root.module = Guard
    current = root
    for _ in range(1200):
        child = DecisionTreeElement("Guard", current)
        child.module = Guard
        current.add_child_element(child, "GO")
        current = child
    action = ActionTreeElement("Idle", current)
    action.module = Idle
    current.add_child_element(action, "GO")
    blackboard = Blackboard()
    blackboard.results["Guard"] = "GO"
    dsd = DSD(blackboard)
    dsd.set_start_element(root)
    dsd.update()
    assert len(dsd.stack) == 1202
    assert blackboard.events[-1] == ("perform", "Idle")
    assert dsd.last_error is None


def test_recursive_update_is_reported_without_executing_another_tick(monkeypatch):
    dsd = make_dsd("-->\n$Chooser\n    GO --> @Idle\n")
    dsd.blackboard.results["Chooser"] = "GO"
    dsd.update()
    monkeypatch.setattr(dsd.stack[-1][1], "perform", lambda: dsd.update())
    dsd.update()
    assert isinstance(dsd.last_error, RuntimeError)
    assert "already running" in str(dsd.last_error)
    assert stack_names(dsd) == ["Chooser", "Idle"]


def test_debug_topic_without_node_does_not_disable_execution():
    dsd = make_dsd("-->\n$Chooser\n    GO --> @Idle\n", debug_topic="debug")
    dsd.blackboard.results["Chooser"] = "GO"
    dsd.update()
    assert ("perform", "Idle") in dsd.blackboard.events
    assert not dsd.debug_active
    assert dsd.last_error is None


def test_debug_failure_cannot_prevent_action_execution(monkeypatch):
    dsd = make_dsd("-->\n$Chooser\n    GO --> @Idle\n")
    dsd.blackboard.results["Chooser"] = "GO"

    def fail():
        raise ValueError("debug failure")

    monkeypatch.setattr(dsd, "debug_publish_stack", fail)
    dsd.update()
    dsd.update()
    assert dsd.blackboard.events.count(("perform", "Idle")) == 2
    assert isinstance(dsd.last_error, ValueError)


def test_debug_publishes_state_after_action_pop():
    node = FakeNode()
    dsd = make_dsd("-->\n$Chooser\n    GO --> @Idle\n", node=node, debug_topic="debug")
    dsd.blackboard.results["Chooser"] = "GO"
    dsd.update()
    dsd.pop()
    dsd.blackboard.results["Chooser"] = "INVALID"
    dsd.update()
    stack = json.loads(node.publishers["debug/dsd_stack"].messages[-1].data)
    assert stack["next"] is None
    assert node.publishers["debug/dsd_current_action"].messages[-1].data == ""
