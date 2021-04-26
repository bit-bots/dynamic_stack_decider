from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class InitTestAction(AbstractActionElement):
    def perform(self, reevaluate=False):
        pass


class InitTestDecision(AbstractDecisionElement):
    def perform(self, reevaluate=False):
        pass
