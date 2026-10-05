"""Base class for outcome-producing decisions."""

from __future__ import annotations

from abc import abstractmethod

from dynamic_stack_decider.abstract_stack_element import AbstractStackElement
from dynamic_stack_decider.types import BlackboardT, ElementData


class AbstractDecisionElement(AbstractStackElement[BlackboardT]):
    """Select a branch from the blackboard; reevaluation should avoid hardware side effects."""

    _reevaluate = False

    def get_reevaluate(self) -> bool:
        """Request reevaluation while this decision is below the current frame."""
        return self._reevaluate

    def repr_dict(self) -> ElementData:
        return {"type": "decision", "name": self.name, "debug_data": self._debug_data}

    @abstractmethod
    def perform(self, reevaluate: bool = False) -> str:
        """Return a non-empty outcome matching a branch or handled by ``ELSE``."""
        raise NotImplementedError(f"Implement perform() in {self.name}")
