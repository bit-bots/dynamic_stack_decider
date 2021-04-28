from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class AmbiguousDecision(AbstractDecisionElement):
    def perform(self, reevaluate=False):
        return "TEST"
