"""
Tests for the parser's error handling and less common code paths: malformed indentation, invalid
element/parameter syntax, invalid sequences and subtree references, and node-provided (``%``)
parameters.
"""

import os
import tempfile

import pytest
from dynamic_stack_decider.parser import DsdParser, ParseError


def parse(text, node=None):
    with tempfile.NamedTemporaryFile("w", suffix=".dsd", delete=False) as f:
        f.write(text)
        path = f.name
    try:
        return DsdParser(node).parse(path)
    finally:
        os.unlink(path)


def test_indent_not_multiple_of_four():
    with pytest.raises(ParseError, match="Indent is not a multiple of 4"):
        parse("-->T\n$Chooser\n   GO --> @FirstAction\n")


def test_element_neither_action_nor_decision():
    with pytest.raises(ParseError, match="neither an action nor a decision"):
        parse("-->T\n$Chooser\n    GO --> FirstAction\n")


def test_root_element_must_start_with_marker():
    with pytest.raises(ParseError, match="start with either"):
        parse("-->T\nFirstDecision\n")


def test_invalid_parameter_list():
    with pytest.raises(ParseError, match="Invalid parameter list"):
        parse("-->T\n$Chooser\n    GO --> @FirstAction + keywithoutvalue\n")


def test_parameter_value_with_space():
    with pytest.raises(ParseError, match="should not contain spaces"):
        parse("-->T\n$Chooser\n    GO --> @FirstAction + key:val ue\n")


def test_decision_not_last_in_sequence():
    with pytest.raises(ParseError, match="Only actions may be followed"):
        parse("-->T\n$Chooser\n    GO --> @FirstAction, $Guard, @SecondAction\n")


def test_subtree_not_defined():
    with pytest.raises(ParseError, match="not defined"):
        parse("-->T\n$Chooser\n    GO --> #DoesNotExist\n")


def test_subtree_invalid_parameters():
    with pytest.raises(ParseError, match="Invalid parameters specified"):
        parse("#Sub + key1\n@FirstAction + p:*key1\n-->T\n$Chooser\n    GO --> #Sub + wrongkey:value\n")


def test_subtree_unknown_reference():
    with pytest.raises(ParseError, match="Unknown reference"):
        parse(
            # The subtree declares parameter 'key1' but the action references the unknown '*missing'
            "#Sub + key1\n@FirstAction + p:*missing\n-->T\n$Chooser\n    GO --> #Sub + key1:value\n"
        )


def test_subtree_used_as_root_of_another_subtree():
    # When a subtree body starts by referencing another subtree, that subtree becomes its root
    tree = parse("#Inner\n@FirstAction\n#Outer\nFOO --> #Inner\n-->T\n$Chooser\n    GO --> #Outer\n")
    assert tree.root_element.get_child("GO").name == "FirstAction"


class _FakeParameter:
    def __init__(self, value):
        self.value = value


class _FakeNode:
    def __init__(self, values):
        self._values = values

    def get_parameter(self, name):
        return _FakeParameter(self._values[name])


def test_node_provided_parameter():
    node = _FakeNode({"speed": 5})
    tree = parse("-->T\n$Chooser\n    GO --> @FirstAction + key:%speed\n", node=node)
    assert tree.root_element.get_child("GO").parameters == {"key": 5}


def test_node_provided_parameter_without_node_is_rejected():
    with pytest.raises(ParseError, match="requires a ROS node"):
        parse("-->T\n$Chooser\n    GO --> @FirstAction + key:%speed\n", node=None)


@pytest.mark.parametrize(
    ("source", "message"),
    [
        ("", "Missing behavior entry"),
        ("@Idle\n", "Expected a subtree declaration"),
        ("-->\n", "no root element"),
        ("-->\n@Idle\n-->\n@Run\n", "Duplicate behavior entry"),
        ("-->\n@Idle\n@Run\n", "Duplicate root"),
        ("-->\n    @Idle\n", "Root element must not be indented"),
        ("-->\n$Guard\n        GO --> @Idle\n", "Invalid indentation"),
        ("-->\n$Guard\n\tGO --> @Idle\n", "not tabs"),
        ("-->\n$Guard\n    GO --> @Idle\n        BAD --> @Run\n", "Invalid indentation"),
        (
            "#Sub\n$Guard\n    GO --> @Idle\n-->\n$Chooser\n    GO --> #Sub\n        BAD --> @Run\n",
            "Invalid indentation",
        ),
        ("-->\n$Guard\n    GO --> @StepA, @Idle\n        BAD --> @Run\n", "Invalid indentation"),
        ("-->\n$Guard\n    GO --> @Idle\n    GO --> @Run\n", "Duplicate outcome"),
        ("-->\n$Guard\n    --> @Idle\n", "outcome must not be empty"),
        ("-->\n$Guard\n", "has no branches"),
        ("#Sub\n@Idle\n#Sub\n@Run\n-->\n@Idle\n", "Duplicate subtree"),
        ("#Sub\n-->\n@Idle\n", "no root element"),
        ("#Sub + x + x\n@Idle\n-->\n@Idle\n", "Duplicate subtree parameter"),
        ("-->\n@Idle + x:1 + x:2\n", "Duplicate parameter"),
        ("-->\n@Idle + x:1 + x:*other\n", "Duplicate parameter"),
        ("-->\n@Idle + x:*missing\n", "Unknown reference"),
        ("-->\n@Idle + x:[\n", "Invalid YAML parameter"),
        ("-->\n@Idle + x:\n", "must not be empty"),
        ("-->\n@Idle + :1\n", "must not be empty"),
        ("-->\n@\n", "name must not be empty"),
        ("-->\n@Idle + x:*\n", "reference must not be empty"),
        ("#Sub\n$Guard\n    A --> @Idle\n    B --> #Sub\n-->\n@Idle\n", "not defined"),
        ("-->\n@Idle\n//** never closed\n", "Unterminated block comment"),
    ],
)
def test_malformed_structure_is_rejected(source, message):
    with pytest.raises(ParseError, match=message):
        parse(source)


def test_error_has_source_location(tmp_path):
    path = tmp_path / "broken.dsd"
    path.write_text("-->\n$Guard\n    GO --> @Idle\n    GO --> @Run\n")
    with pytest.raises(ParseError) as caught:
        DsdParser().parse(path)
    assert caught.value.path == path
    assert caught.value.line == 4
    assert f"{path}:4:" in str(caught.value)


def test_sibling_after_chain_decision_attaches_to_outer_decision():
    tree = parse("-->\n$Chooser\n    FIRST --> @StepA, $Guard\n        GO --> @Idle\n    SECOND --> @Run\n")
    assert set(tree.root_element.children) == {"FIRST", "SECOND"}
    assert set(tree.root_element.get_child("FIRST").next_element.children) == {"GO"}


def test_resolved_subtree_references_do_not_leak_into_outer_calls():
    tree = parse(
        "#Inner + value\n@Idle + x:*value\n"
        "#Outer + other\n$Guard\n    GO --> #Inner + value:fixed\n"
        "    ELSE --> @Run + x:*other\n"
        "-->\n$Chooser\n    GO --> #Outer + other:outer\n"
    )
    inner = tree.root_element.get_child("GO").get_child("GO")
    assert inner.parameters == {"x": "fixed"}
    assert inner.unset_parameters == {}


def test_parameterized_root_alias_is_copied_and_resolved():
    tree = parse(
        "#Inner + value\n@Idle + x:*value\n"
        "#Outer + other\nALIAS --> #Inner + value:*other\n"
        "-->\n$Chooser\n    GO --> #Outer + other:resolved\n"
    )
    assert tree.root_element.get_child("GO").parameters == {"x": "resolved"}


def test_copied_subtree_root_has_its_calling_parent():
    tree = parse("#Sub\n@Idle\n-->\n$Chooser\n    GO --> #Sub\n")
    child = tree.root_element.get_child("GO")
    assert child.parent is tree.root_element
    assert child.source_line == 2


def test_sibling_after_nested_decision_closes_deeper_indentation():
    with pytest.raises(ParseError, match="Invalid indentation"):
        parse("-->\n$Chooser\n    FIRST --> $Guard\n        GO --> @Idle\n    SECOND --> @Run\n        BAD --> @Idle\n")
