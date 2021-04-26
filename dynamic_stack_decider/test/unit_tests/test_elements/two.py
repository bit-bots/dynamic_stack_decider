from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class TwoTestAction(AbstractActionElement):
    def perform(self, reevaluate=False):
        pass


class TwoTestDecision(AbstractDecisionElement):
    def perform(self, reevaluate=False):
        pass
