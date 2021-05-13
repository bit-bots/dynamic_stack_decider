import os
import unittest

from dynamic_stack_decider.parser import parse as parse_dsd
from dynamic_stack_decider.tree import DecisionTreeElement, ActionTreeElement, SequenceTreeElement


class ParserTest(unittest.TestCase):
    def setUp(self):
        self.tree = parse_dsd(os.path.join(os.path.dirname(__file__), 'test.dsd'))

    def test_root_element(self):
        root_element = self.tree.root_element
        self.assertTrue(isinstance(root_element, DecisionTreeElement))
        self.assertEqual(root_element.name, 'FirstDecision')

    def test_possible_results(self):
        self.assertSetEqual(set(self.tree.root_element.children.keys()),
                            {'ACTION', 'DECISION', 'SUBBEHAVIOR', 'SEQUENCE', 'PARAMETERS',
                             'LINE_COMMENT', 'BLOCK_COMMENT', 'COMPLICATED_COMMENT',
                             'MULTIPLE_PARAMETERS', 'SECOND_SUBBEHAVIOR_1', 'SECOND_SUBBEHAVIOR_2',
                             'PARAMETER_DECISION', 'PARAMETER_SUBBEHAVIOR', 'NESTED_PARAMETER_SUBBEHAVIOR'})

    def test_following_elements(self):
        first_child = self.tree.root_element.get_child('ACTION')
        self.assertEqual(first_child.name, 'FirstAction')
        self.assertTrue(isinstance(first_child, ActionTreeElement))

        second_child = self.tree.root_element.get_child('DECISION')
        self.assertEqual(second_child.name, 'SecondDecision')
        self.assertTrue(isinstance(second_child, DecisionTreeElement))

    def test_nested_decision(self):
        decision_child = self.tree.root_element.get_child('DECISION')
        self.assertSetEqual(set(decision_child.children.keys()), {'FIRST', 'SECOND'})
        self.assertEqual(decision_child.get_child('FIRST').name, 'FirstAction')
        self.assertTrue(isinstance(decision_child.get_child('FIRST'), ActionTreeElement))
        self.assertEqual(decision_child.get_child('SECOND').name, 'SecondAction')
        self.assertTrue(isinstance(decision_child.get_child('SECOND'), ActionTreeElement))

    def test_sub_behavior(self):
        sub_behavior_root_decision = self.tree.root_element.get_child('SUBBEHAVIOR')
        self.assertEqual(sub_behavior_root_decision.name, 'ThirdDecision')
        self.assertTrue(isinstance(sub_behavior_root_decision, DecisionTreeElement))
        self.assertSetEqual(set(sub_behavior_root_decision.children.keys()), {'FIRST', 'SECOND'})
        self.assertEqual(sub_behavior_root_decision.get_child('FIRST').name, 'FirstAction')
        self.assertTrue(isinstance(sub_behavior_root_decision.get_child('FIRST'), ActionTreeElement))
        self.assertEqual(sub_behavior_root_decision.get_child('SECOND').name, 'SecondAction')
        self.assertTrue(isinstance(sub_behavior_root_decision.get_child('SECOND'), ActionTreeElement))

    def test_sequence_element(self):
        sequence_element = self.tree.root_element.get_child('SEQUENCE')
        self.assertTrue(isinstance(sequence_element, SequenceTreeElement))
        self.assertEqual(len(sequence_element.action_elements), 2)
        first_action = sequence_element.action_elements[0]
        self.assertEqual(first_action.name, 'FirstAction')
        self.assertTrue(isinstance(first_action, ActionTreeElement))
        second_action = sequence_element.action_elements[1]
        self.assertEqual(second_action.name, 'SecondAction')
        self.assertTrue(isinstance(second_action, ActionTreeElement))

    def test_action_parameters(self):
        parameter_element = self.tree.root_element.get_child('PARAMETERS')
        self.assertEqual(parameter_element.name, 'FirstAction')
        self.assertTrue(isinstance(parameter_element, ActionTreeElement))
        self.assertDictEqual(parameter_element.parameters, {'key': 'value'})

    def test_decision_parameters(self):
        parameter_element = self.tree.root_element.get_child('PARAMETER_DECISION')
        self.assertEqual(parameter_element.name, 'FirstDecision')
        self.assertTrue(isinstance(parameter_element, DecisionTreeElement))
        self.assertDictEqual(parameter_element.parameters, {'key': 'value'})

    def test_line_comment(self):
        comment_element = self.tree.root_element.get_child('LINE_COMMENT')
        self.assertEqual(comment_element.name, 'FirstAction')
        self.assertTrue(isinstance(comment_element, ActionTreeElement))

    def test_block_comment(self):
        comment_element = self.tree.root_element.get_child('BLOCK_COMMENT')
        self.assertEqual(comment_element.name, 'FirstAction')
        self.assertTrue(isinstance(comment_element, ActionTreeElement))
        self.assertDictEqual(comment_element.parameters, {'key': 'value'})

    def test_complicated_comment(self):
        comment_element = self.tree.root_element.get_child('COMPLICATED_COMMENT')
        self.assertEqual(comment_element.name, 'FirstAction')
        self.assertTrue(isinstance(comment_element, ActionTreeElement))

    def test_multiple_parameters(self):
        parameter_element = self.tree.root_element.get_child('MULTIPLE_PARAMETERS')
        self.assertEqual(parameter_element.name, 'FirstAction')
        self.assertTrue(isinstance(parameter_element, ActionTreeElement))
        self.assertDictEqual(parameter_element.parameters,
                             {'key1': 'value1', 'key2': 'value2'})

    def test_multiple_subbehavior_references(self):
        sub_behavior_1_root_decision = self.tree.root_element.get_child('SECOND_SUBBEHAVIOR_1')
        sub_behavior_2_root_decision = self.tree.root_element.get_child('SECOND_SUBBEHAVIOR_2')
        self.assertEqual(sub_behavior_1_root_decision.name, sub_behavior_2_root_decision.name)
        self.assertEqual(sub_behavior_1_root_decision.activation_reason, 'SECOND_SUBBEHAVIOR_1')
        self.assertEqual(sub_behavior_2_root_decision.activation_reason, 'SECOND_SUBBEHAVIOR_2')

    def test_parameter_subbehavior(self):
        parameter_subbehavior_decision = self.tree.root_element.get_child('PARAMETER_SUBBEHAVIOR')
        first_action = parameter_subbehavior_decision.get_child('FIRST')
        self.assertEqual(first_action.parameters, {'key': 'value1'})
        action_sequence = parameter_subbehavior_decision.get_child('SECOND')
        self.assertEqual(action_sequence.action_elements[0].parameters, {'key': 'value2'})
        self.assertEqual(action_sequence.action_elements[1].parameters, {'key': 'value2'})

    def test_nested_parameter_subbehavior(self):
        subbehavior_decision = self.tree.root_element.get_child('NESTED_PARAMETER_SUBBEHAVIOR')
        inner_subbehavior_decision = subbehavior_decision.get_child('FIRST')
        first_action = inner_subbehavior_decision.get_child('FIRST')
        self.assertEqual(first_action.parameters, {'key': 'nested1'})
        action_sequence = inner_subbehavior_decision.get_child('SECOND')
        self.assertEqual(action_sequence.action_elements[0].parameters, {'key': 'nested2'})
        self.assertEqual(action_sequence.action_elements[1].parameters, {'key': 'nested2'})


if __name__ == '__main__':
    try:
        import xmlrunner
        with open('report.xml', 'wb') as output:
            unittest.main(
                testRunner=xmlrunner.XMLTestRunner(output=output)
            )
    except ImportError:
        unittest.main()
