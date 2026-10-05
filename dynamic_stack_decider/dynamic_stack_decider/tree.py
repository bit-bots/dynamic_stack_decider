from __future__ import annotations

from typing import TYPE_CHECKING, Generic

from dynamic_stack_decider.types import BlackboardT, ParameterReferences, Parameters, TreeData

if TYPE_CHECKING:
    from dynamic_stack_decider.abstract_stack_element import AbstractStackElement


class Tree(Generic[BlackboardT]):
    """A parsed behavior or subtree definition."""

    def __init__(self) -> None:
        self.root_element: AbstractTreeElement[BlackboardT] | None = None
        self.parameter_list: list[str] = []

    def set_root_element(self, element: AbstractTreeElement[BlackboardT]) -> None:
        """Set the root blueprint of this definition."""
        self.root_element = element

    def __repr__(self) -> str:
        return repr(self.root_element)

    def repr_dict(self) -> TreeData:
        """Serialize this blueprint for the debug protocol."""
        if self.root_element is None:
            raise ValueError("Tree has no root element")
        return self.root_element.repr_dict()


class AbstractTreeElement(Generic[BlackboardT]):
    """A blueprint bound to an element class before activation."""

    def __init__(
        self,
        name: str,
        parent: AbstractTreeElement[BlackboardT] | None,
        parameters: Parameters | None = None,
        unset_parameters: ParameterReferences | None = None,
    ) -> None:
        self.source_file: str | None = None
        self.source_line: int | None = None
        self.name = name
        self.parent = parent
        self.parameters = parameters or dict()
        self.unset_parameters = unset_parameters or dict()
        self.module: type[AbstractStackElement[BlackboardT]] | None = None
        self.activation_reason: str | None = None

    def get_child(self, activating_result: str) -> AbstractTreeElement[BlackboardT] | None:
        """Return the selected child; actions have no decision children."""
        return None

    def set_activation_reason(self, reason: str | None) -> None:
        """Record the selecting branch, propagating it along an action chain."""
        self.activation_reason = reason

    def repr_dict(self) -> TreeData:
        """Serialize this blueprint for the debug protocol."""
        return {
            # IDs distinguish repeated class names within this loaded tree.
            "id": id(self),
            "name": self.name,
            "parameters": self.parameters,
        }


class DecisionTreeElement(AbstractTreeElement[BlackboardT]):
    """Map outcomes to child blueprints, with optional ELSE fallback."""

    def __init__(
        self,
        name: str,
        parent: AbstractTreeElement[BlackboardT] | None,
        parameters: Parameters | None = None,
        unset_parameters: ParameterReferences | None = None,
    ) -> None:
        super().__init__(name, parent, parameters, unset_parameters)

        self.children: dict[str, AbstractTreeElement[BlackboardT]] = dict()

    def add_child_element(self, element: AbstractTreeElement[BlackboardT], activating_result: str) -> None:
        """Attach a blueprint to a decision outcome."""
        self.children[activating_result] = element
        element.set_activation_reason(activating_result)

    def get_child(self, activating_result: str | None) -> AbstractTreeElement[BlackboardT]:
        """Return the matching child or ELSE; reject invalid or unmapped outcomes."""
        if not isinstance(activating_result, str) or not activating_result:
            raise ValueError(f"Decision {self.name!r} must return a non-empty string, got {activating_result!r}")

        if activating_result in self.children:
            return self.children[activating_result]
        elif "ELSE" in self.children:
            return self.children["ELSE"]
        else:
            raise KeyError(f"{activating_result} does not lead to a child of {str(self)} and no ELSE was specified")

    def __repr__(self) -> str:
        r = "$" + self.name + f" ({self.parameters}): "
        for result, child in self.children.items():
            r += result + ": {" + repr(child) + "} "
        return r

    def __str__(self) -> str:
        return self.name

    def repr_dict(self) -> TreeData:
        """Serialize this blueprint for the debug protocol."""
        result = super().repr_dict()
        result["type"] = "decision"
        result["children"] = {result: child.repr_dict() for result, child in self.children.items()}
        return result


class ActionTreeElement(AbstractTreeElement[BlackboardT]):
    """An action blueprint whose successor is initialized on completion."""

    def __init__(
        self,
        name: str,
        parent: AbstractTreeElement[BlackboardT] | None,
        parameters: Parameters | None = None,
        unset_parameters: ParameterReferences | None = None,
    ) -> None:
        super().__init__(name, parent, parameters, unset_parameters)
        self.next_element: AbstractTreeElement[BlackboardT] | None = None

    def set_activation_reason(self, reason: str | None) -> None:
        """Record the selecting branch, propagating it along an action chain."""
        self.activation_reason = reason
        # All elements of a chain share the activation reason of the head so that a
        # reevaluated decision below the chain does not consider its result changed
        # while the chain is still running.
        if self.next_element is not None:
            self.next_element.set_activation_reason(reason)

    def __repr__(self) -> str:
        if self.next_element is not None:
            return f"@{self.name} ({self.parameters}) -> {repr(self.next_element)}"
        return f"@{self.name} ({self.parameters})"

    def __str__(self) -> str:
        return self.name

    def repr_dict(self) -> TreeData:
        """Serialize this blueprint for the debug protocol."""
        result = super().repr_dict()
        result["type"] = "action"
        result["next"] = self.next_element.repr_dict() if self.next_element is not None else None
        return result
