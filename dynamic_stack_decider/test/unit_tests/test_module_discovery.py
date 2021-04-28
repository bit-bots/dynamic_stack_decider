from pathlib import Path

from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from dynamic_stack_decider.dsd import discover_elements, DSD
from bitbots_test.test_case import TestCase

project_dir = Path(__file__).parent / "test_project"


class TestAction(AbstractActionElement):
    def perform(self, reevaluate=False):
        pass


class ModuleDiscoveryTestCase(TestCase):
    def test_single_file(self):
        # execution
        elements = discover_elements(str(project_dir / "actions" / "test_action.py"))

        # verification
        self.assertEqual(list(elements.keys()), ["TestAction"])

    def test_explicit_init_py_file(self):
        # execution
        elements = discover_elements(str(project_dir / "actions" / "__init__.py"))

        # verification
        self.assertEqual(list(elements.keys()), ["InitAction"])

    def test_directory(self):
        # execution
        elements = discover_elements(str(project_dir / "actions"))

        # verification
        self.assertEqual(list(elements.keys()), ["InitAction", "AmbiguousAction", "TestAction"])

    def test_directory_traversal(self):
        # execution
        elements = discover_elements(str(project_dir))

        # verification
        self.assertEqual(list(elements.keys()), ["InitAction", "AmbiguousAction", "TestAction", "InitDecision", "AmbiguousDecision", "TestDecision"])

    def test_file_with_unrelated_class(self):
        # execution
        elements = discover_elements(__file__)

        # verification
        self.assertEqual(list(elements.keys()), ["TestAction"])


class ModuleRegistrationTestCase(TestCase):
    def setUp(self) -> None:
        super().setUp()
        self.dsd = DSD({})

    def test_register_actions(self):
        with self.subTest("project_dir"):
            # execution
            self.dsd.register_actions(str(project_dir))
            # verification
            self.assertEqual(list(self.dsd.actions.keys()), ["InitAction", "AmbiguousAction", "TestAction"])

        self.setUp()
        with self.subTest("project_dir / actions"):
            # execution
            self.dsd.register_actions(str(project_dir / "actions"))
            # verification
            self.assertEqual(list(self.dsd.actions.keys()), ["InitAction", "AmbiguousAction", "TestAction"])

    def test_register_decisions(self):
        with self.subTest("project_dir"):
            # execution
            self.dsd.register_decisions(str(project_dir))
            # verification
            self.assertEqual(list(self.dsd.decisions.keys()), ["InitDecision", "AmbiguousDecision", "TestDecision"])

        self.setUp()
        with self.subTest("project_dir / decisions"):
            # execution
            self.dsd.register_decisions(str(project_dir / "decisions"))
            # verification
            self.assertEqual(list(self.dsd.decisions.keys()), ["InitDecision", "AmbiguousDecision", "TestDecision"])



if __name__ == "__main__":
    from bitbots_test import run_unit_tests
    run_unit_tests(ModuleDiscoveryTestCase)
