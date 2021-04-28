from pathlib import Path

from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from dynamic_stack_decider.dsd import discover_elements
from bitbots_test.test_case import TestCase


class TestAction(AbstractActionElement):
    def perform(self, reevaluate=False):
        pass


class ModuleDiscoveryTestCase(TestCase):
    project_dir = Path(__file__).parent / "test_project"

    def test_single_file(self):
        # execution
        elements = discover_elements(str(self.project_dir / "actions" / "test_action.py"))

        # verification
        self.assertEqual(list(elements.keys()), ["TestAction"])

    def test_explicit_init_py_file(self):
        # execution
        elements = discover_elements(str(self.project_dir / "actions" / "__init__.py"))

        # verification
        self.assertEqual(list(elements.keys()), ["InitAction"])

    def test_directory(self):
        # execution
        elements = discover_elements(str(self.project_dir / "actions"))

        # verification
        self.assertEqual(list(elements.keys()), ["InitAction", "AmbiguousAction", "TestAction"])

    def test_directory_traversal(self):
        # execution
        elements = discover_elements(str(self.project_dir))

        # verification
        self.assertEqual(list(elements.keys()), ["InitAction", "AmbiguousAction", "TestAction", "InitDecision", "AmbiguousDecision", "TestDecision"])

    def test_file_with_unrelated_class(self):
        # execution
        elements = discover_elements(__file__)

        # verification
        self.assertEqual(list(elements.keys()), ["TestAction"])


if __name__ == "__main__":
    from bitbots_test import run_unit_tests
    run_unit_tests(ModuleDiscoveryTestCase)
