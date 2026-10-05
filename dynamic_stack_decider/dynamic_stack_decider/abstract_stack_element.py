"""Shared lifecycle and debugging helpers for actions and decisions."""

from __future__ import annotations

import json
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Generic

from dynamic_stack_decider.logger import get_logger
from dynamic_stack_decider.types import BlackboardT, ElementData, Parameters

if TYPE_CHECKING:
    from dynamic_stack_decider.dsd import DSD


class AbstractStackElement(Generic[BlackboardT], ABC):
    def __init__(self, blackboard: BlackboardT, dsd: DSD[BlackboardT], parameters: Parameters) -> None:
        self._dsd = dsd
        self.blackboard = blackboard
        self.parameters = parameters
        self._debug_data: dict[str, object] = {}

    @property
    def name(self) -> str:
        return type(self).__name__

    def pop(self) -> None:
        """Complete this frame. Return immediately afterward to avoid using a discarded instance."""
        self._dsd.pop()

    def on_pop(self) -> None:
        """Release resources when completed, preempted, interrupted, or replaced by another behavior."""

    @abstractmethod
    def perform(self, reevaluate: bool = False) -> str | None:
        """Execute the element, or check a decision during reevaluation."""
        raise NotImplementedError(f"Implement perform() in {self.name}")

    def interrupt(self) -> None:
        """Restart the behavior. Return immediately afterward."""
        self._dsd.interrupt()

    def publish_debug_data(self, label: str, data: object) -> None:
        """Store JSON-compatible debug data, representing unsupported values as strings."""
        try:
            data = json.loads(json.dumps(data, default=str, allow_nan=False))
        except (TypeError, ValueError):
            data = str(data)
        get_logger().debug(f"{label}: {data}")
        self._debug_data[label] = data

    def clear_debug_data(self) -> None:
        self._debug_data.clear()

    def repr_dict(self) -> ElementData:
        return {"type": "abstract", "name": self.name, "debug_data": self._debug_data}
