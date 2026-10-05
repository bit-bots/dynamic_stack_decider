"""Base class for persistent actions."""

from __future__ import annotations

from abc import abstractmethod
from typing import TYPE_CHECKING

from dynamic_stack_decider.abstract_stack_element import AbstractStackElement
from dynamic_stack_decider.types import BlackboardT, ElementData, Parameters

if TYPE_CHECKING:
    from dynamic_stack_decider.dsd import DSD


class AbstractActionElement(AbstractStackElement[BlackboardT]):
    """Perform work until completion or preemption; implement cleanup in ``on_pop``."""

    def __init__(self, blackboard: BlackboardT, dsd: DSD[BlackboardT], parameters: Parameters) -> None:
        super().__init__(blackboard, dsd, parameters)
        self.never_reevaluate = not parameters.get("r", True) or not parameters.get("reevaluate", True)

    def do_not_reevaluate(self) -> None:
        """Suppress the next reevaluation, for example while finishing an atomic motion."""
        self._dsd.set_do_not_reevaluate()

    def repr_dict(self) -> ElementData:
        return {"type": "action", "name": self.name, "debug_data": self._debug_data}

    @abstractmethod
    def perform(self, reevaluate: bool = False) -> None:
        """Perform one tick of work. Complete with ``return self.pop()``."""
        raise NotImplementedError(f"Implement perform() in {self.name}")
