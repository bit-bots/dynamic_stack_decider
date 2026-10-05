"""Shared parameter, blackboard, and debug representation types."""

from __future__ import annotations

from typing import Literal, TypedDict, TypeVar

BlackboardT = TypeVar("BlackboardT")
Parameters = dict[str, object]
ParameterReferences = dict[str, str]


class ElementData(TypedDict):
    type: Literal["abstract", "action", "decision"]
    name: str
    debug_data: dict[str, object]


class StackData(ElementData):
    id: int
    activation_reason: str | None
    next: StackData | None


class _TreeFields(TypedDict):
    id: int
    name: str
    parameters: Parameters


class TreeData(_TreeFields, total=False):
    type: Literal["action", "decision"]
    children: dict[str, TreeData]
    next: TreeData | None
