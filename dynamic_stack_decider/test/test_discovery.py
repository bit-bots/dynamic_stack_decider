"""
Tests for element discovery and registration (``discover_elements``, ``register_actions``,
``register_decisions``). These exercise the directory, single-file and ``__init__.py`` code paths,
the type filtering and the resilience against modules that fail to import.
"""

import os

import pytest
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement
from dynamic_stack_decider.dsd import DSD, discover_elements

FIXTURES = os.path.join(os.path.dirname(__file__), "fixture_elements")


def test_discover_from_directory():
    elements = discover_elements(FIXTURES)
    # Classes from submodules and the package __init__ are found
    assert "FixtureActionA" in elements
    assert "FixtureActionB" in elements
    assert "FixtureDecision" in elements
    assert "InitAction" in elements
    # Plain classes that are not stack elements are ignored
    assert "NotAnElement" not in elements


def test_discover_from_single_file():
    elements = discover_elements(os.path.join(FIXTURES, "actions.py"))
    assert {"FixtureActionA", "FixtureActionB"}.issubset(elements)
    # Elements from other modules are not included
    assert "FixtureDecision" not in elements
    assert "NotAnElement" not in elements


def test_discover_from_init_file(monkeypatch):
    # The __init__.py path resolves the module name from the parent directory, which requires the
    # package's parent (the test directory) to be importable.
    monkeypatch.syspath_prepend(os.path.dirname(FIXTURES))
    elements = discover_elements(os.path.join(FIXTURES, "__init__.py"))
    assert "InitAction" in elements


def test_discover_invalid_path_raises():
    with pytest.raises(ValueError):
        discover_elements(os.path.join(FIXTURES, "this_path_does_not_exist"))


def test_discover_skips_unimportable_module():
    # broken.py raises on import; discovery must log and continue, still returning the good classes
    elements = discover_elements(FIXTURES)
    assert "FixtureActionA" in elements


def test_register_actions_filters_to_actions():
    dsd = DSD(blackboard=object())
    dsd.register_actions(FIXTURES)
    assert "FixtureActionA" in dsd.actions
    assert "FixtureActionB" in dsd.actions
    # Decisions are filtered out
    assert "FixtureDecision" not in dsd.actions
    assert all(issubclass(cls, AbstractActionElement) for cls in dsd.actions.values())


def test_register_decisions_filters_to_decisions():
    dsd = DSD(blackboard=object())
    dsd.register_decisions(FIXTURES)
    assert "FixtureDecision" in dsd.decisions
    # Actions are filtered out
    assert "FixtureActionA" not in dsd.decisions
    assert all(issubclass(cls, AbstractDecisionElement) for cls in dsd.decisions.values())


def test_init_file_discovery_does_not_require_existing_sys_path(tmp_path):
    import sys

    package = tmp_path / "dsd_unique_fixture_package"
    package.mkdir()
    path = package / "__init__.py"
    path.write_text(
        "from dynamic_stack_decider import AbstractActionElement\n"
        "class DiscoveredAction(AbstractActionElement):\n"
        "    def perform(self, reevaluate=False): pass\n"
    )
    original_path = list(sys.path)
    try:
        assert "DiscoveredAction" in discover_elements(path)
        assert sys.path == original_path
    finally:
        sys.modules.pop(package.name, None)


def test_discovery_restores_sys_path_if_package_walk_fails(monkeypatch):
    import sys

    import dynamic_stack_decider.dsd as runtime

    original_path = list(sys.path)

    def fail(*args, **kwargs):
        raise RuntimeError("package walk failed")

    monkeypatch.setattr(runtime.pkgutil, "walk_packages", fail)
    with pytest.raises(RuntimeError, match="package walk failed"):
        discover_elements(FIXTURES)
    assert sys.path == original_path
