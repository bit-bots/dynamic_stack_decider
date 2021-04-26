from pathlib import Path

from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from dynamic_stack_decider.dsd import discover_elements
from bitbots_test.test_case import TestCase


class TestAction(AbstractActionElement):
    def perform(self, reevaluate=False):
        pass


class ModuleDiscoveryTestCase(TestCase):
    element_dir = Path(__file__).parent / "test_elements"

    def test_single_file(self):
        # execution
        elements = discover_elements(str(self.element_dir / "one.py"))

        # verification
        self.assertEqual(list(elements.keys()), ["OneTestAction", "OneTestDecision"])

    def test_explicit_init_py_file(self):
        # execution
        elements = discover_elements(str(self.element_dir / "__init__.py"))

        # verification
        self.assertEqual(list(elements.keys()), ["InitTestAction", "InitTestDecision"])

    def test_directory(self):
        # execution
        elements = discover_elements(str(self.element_dir))

        # verification
        self.assertEqual(list(elements.keys()), ["InitTestAction", "InitTestDecision", "OneTestAction", "OneTestDecision", "TwoTestAction", "TwoTestDecision"])

    def test_file_with_unrelated_class(self):
        # execution
        elements = discover_elements(__file__)

        # verification
        self.assertEqual(list(elements.keys()), ["TestAction"])


if __name__ == "__main__":
    from bitbots_test import run_unit_tests
    run_unit_tests(ModuleDiscoveryTestCase)
