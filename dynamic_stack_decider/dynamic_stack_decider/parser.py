"""Parse the DSD language into decisions and action chains."""

from __future__ import annotations

import copy
import re
from collections.abc import Iterator
from pathlib import Path
from typing import Generic

import yaml
from rclpy.node import Node

from dynamic_stack_decider.tree import AbstractTreeElement, ActionTreeElement, DecisionTreeElement, Tree
from dynamic_stack_decider.types import BlackboardT, ParameterReferences, Parameters


class ParseError(AssertionError):
    """Invalid DSD input, with the source file and line when available."""

    def __init__(self, message: str, path: Path | None = None, line: int | None = None) -> None:
        self.path = path
        self.line = line
        location = f"{path}:{line}" if path is not None else f"line {line}"
        super().__init__(f"{location}: {message}" if line is not None else message)


class DsdParser(Generic[BlackboardT]):
    def __init__(self, node: Node | None = None) -> None:
        self.node = node
        self._source: Path | None = None

    def _error(self, line: int, message: str) -> ParseError:
        return ParseError(message, self._source, line)

    def _lines(self, path: Path) -> Iterator[tuple[int, str]]:
        """Strip comments while preserving indentation on the remaining code."""
        block_start: int | None = None
        with path.open(encoding="utf-8") as stream:
            for line_number, raw_line in enumerate(stream, start=1):
                remaining = raw_line
                parts: list[str] = []
                while remaining:
                    if block_start is not None:
                        _, marker, remaining = remaining.partition("**//")
                        if not marker:
                            break
                        block_start = None
                    else:
                        before, marker, after = remaining.partition("//")
                        parts.append(before)
                        if not marker:
                            break
                        if not after.startswith("**"):
                            break
                        block_start = line_number
                        remaining = after[2:]
                line = "".join(parts).rstrip()
                if line:
                    yield line_number, line
        if block_start is not None:
            raise self._error(block_start, "Unterminated block comment")

    def parse(self, file_path: str | Path) -> Tree[BlackboardT]:
        """Parse and validate a behavior before any implementation is instantiated."""
        self._source = Path(file_path)
        tree: Tree[BlackboardT] = Tree()
        subtrees: dict[str, Tree[BlackboardT]] = {}
        current: Tree[BlackboardT] | None = None
        current_name: str | None = None
        current_header = 1
        parents: dict[int, DecisionTreeElement[BlackboardT]] = {}
        entry_seen = False
        last_line = 1

        for line_number, line in self._lines(self._source):
            last_line = line_number
            whitespace = line[: len(line) - len(line.lstrip())]
            if "\t" in whitespace:
                raise self._error(line_number, "Indentation must use spaces, not tabs")
            indent = len(whitespace)
            if indent % 4:
                raise self._error(line_number, "Indent is not a multiple of 4")
            content = line.lstrip()

            if indent == 0 and (content.startswith("-->") or content.startswith("#")):
                if current is not None:
                    self._validate_tree(current, current_header)
                    if current_name is not None:
                        subtrees[current_name] = current
                parents.clear()
                current_header = line_number
                if content.startswith("-->"):
                    if entry_seen:
                        raise self._error(line_number, "Duplicate behavior entry point")
                    entry_seen = True
                    current = tree
                    current_name = None
                else:
                    tokens = re.split(r"\s*\+\s*", content[1:])
                    name, declared = tokens[0], tokens[1:]
                    if not name or any(not parameter or ":" in parameter for parameter in declared):
                        raise self._error(line_number, "Invalid subtree declaration")
                    if name in subtrees:
                        raise self._error(line_number, f"Duplicate subtree {name!r}")
                    if len(set(declared)) != len(declared):
                        raise self._error(line_number, "Duplicate subtree parameter")
                    current = Tree()
                    current.parameter_list = declared
                    current_name = name
                continue

            if current is None:
                raise self._error(line_number, "Expected a subtree declaration or behavior entry point (-->)")

            if current.root_element is None:
                if indent:
                    raise self._error(line_number, "Root element must not be indented")
                # Preserve the legacy labelled subtree alias syntax: RESULT --> #Subtree.
                if "->" in content:
                    _, content = re.split(r"\s*-?->\s*", content, maxsplit=1)
                    if not content.startswith("#"):
                        raise self._error(line_number, "A root alias must reference a subtree")
                head, tail = self._create_chain(re.split(r"\s*,\s*", content), None, line_number, subtrees)
                current.set_root_element(head)
                if isinstance(tail, DecisionTreeElement) and not content.split(",")[-1].strip().startswith("#"):
                    parents[4] = tail
                continue

            if indent == 0:
                raise self._error(line_number, "Duplicate root element")
            parent = parents.get(indent)
            if parent is None:
                raise self._error(line_number, "Invalid indentation: branches must belong to a decision")
            if "->" not in content:
                raise self._error(line_number, "Expected a decision outcome followed by -->")
            result, call = re.split(r"\s*-?->\s*", content, maxsplit=1)
            if not result:
                raise self._error(line_number, "Decision outcome must not be empty")
            if result in parent.children:
                raise self._error(line_number, f"Duplicate outcome {result!r} on decision {parent.name!r}")
            tokens = re.split(r"\s*,\s*", call)
            head, tail = self._create_chain(tokens, parent, line_number, subtrees)
            parent.add_child_element(head, result)
            parents = {level: decision for level, decision in parents.items() if level <= indent}
            if isinstance(tail, DecisionTreeElement) and not tokens[-1].startswith("#"):
                parents[indent + 4] = tail

        if current is not None:
            self._validate_tree(current, current_header)
        if not entry_seen:
            raise self._error(last_line, "Missing behavior entry point (-->)")
        return tree

    def _validate_tree(self, tree: Tree[BlackboardT], line: int) -> None:
        if tree.root_element is None:
            raise self._error(line, "Definition has no root element")
        pending = [tree.root_element]
        while pending:
            element = pending.pop()
            unknown = set(element.unset_parameters.values()) - set(tree.parameter_list)
            if unknown:
                raise self._error(element.source_line or line, f"Unknown reference: {', '.join(sorted(unknown))}")
            if isinstance(element, DecisionTreeElement):
                if not element.children:
                    raise self._error(element.source_line or line, f"Decision {element.name!r} has no branches")
                pending.extend(element.children.values())
            elif isinstance(element, ActionTreeElement) and element.next_element is not None:
                pending.append(element.next_element)

    def _extract_parameters(self, token: str, lnr: int) -> tuple[str, Parameters, ParameterReferences]:
        tokens = re.split(r"\s*\+\s*", token)
        name = tokens.pop(0)
        if not name:
            raise self._error(lnr, "Element name must not be empty")
        parameters: Parameters = {}
        references: ParameterReferences = {}
        for token in tokens:
            try:
                key, value = token.split(":")
            except ValueError as error:
                raise self._error(lnr, "Invalid parameter list") from error
            if not key or not value:
                raise self._error(lnr, "Parameter key and value must not be empty")
            if key in parameters or key in references:
                raise self._error(lnr, f"Duplicate parameter {key!r}")
            if " " in value:
                raise self._error(lnr, "Parameter values should not contain spaces. Did you forget a comma?")
            if value in ("%", "*"):
                raise self._error(lnr, "Parameter reference must not be empty")
            if value.startswith("%"):
                if self.node is None:
                    raise self._error(lnr, "ROS parameter lookup requires a ROS node")
                try:
                    parameters[key] = self.node.get_parameter(value[1:]).value
                except Exception as error:
                    raise self._error(lnr, f"Cannot resolve ROS parameter {value[1:]!r}: {error}") from error
            elif value.startswith("*"):
                references[key] = value[1:]
            else:
                try:
                    parameters[key] = yaml.safe_load(value)
                except yaml.YAMLError as error:
                    raise self._error(lnr, f"Invalid YAML parameter {key!r}: {error}") from error
        return name, parameters, references

    def _create_tree_element(
        self, token: str, parent: AbstractTreeElement[BlackboardT] | None, lnr: int
    ) -> AbstractTreeElement[BlackboardT]:
        if not token.startswith(("$", "@")):
            raise self._error(
                lnr, "An element has to start with either $ or @; neither an action nor a decision was given"
            )
        name, parameters, references = self._extract_parameters(token[1:], lnr)
        element: AbstractTreeElement[BlackboardT]
        if token.startswith("$"):
            element = DecisionTreeElement(name, parent, parameters, references)
        else:
            element = ActionTreeElement(name, parent, parameters, references)
        element.source_file = str(self._source) if self._source is not None else None
        element.source_line = lnr
        return element

    def _create_chain(
        self,
        tokens: list[str],
        parent: AbstractTreeElement[BlackboardT] | None,
        lnr: int,
        subtrees: dict[str, Tree[BlackboardT]],
    ) -> tuple[AbstractTreeElement[BlackboardT], AbstractTreeElement[BlackboardT]]:
        head: AbstractTreeElement[BlackboardT] | None = None
        previous: AbstractTreeElement[BlackboardT] | None = None
        for index, token in enumerate(tokens):
            if index < len(tokens) - 1 and not token.startswith("@"):
                raise self._error(lnr, f"Only actions may be followed by another element in a sequence, got {token!r}")
            element = (
                self._instantiate_subtree(token, lnr, subtrees)
                if token.startswith("#")
                else self._create_tree_element(token, parent, lnr)
            )
            element.parent = parent
            if previous is None:
                head = element
            elif isinstance(previous, ActionTreeElement):
                previous.next_element = element
            previous = element
        if head is None or previous is None:
            raise self._error(lnr, "Empty action chain")
        return head, previous

    def _lookup_subtree(
        self, call: str, lnr: int, subtrees: dict[str, Tree[BlackboardT]]
    ) -> tuple[Tree[BlackboardT], Parameters, ParameterReferences]:
        name, parameters, references = self._extract_parameters(call[1:], lnr)
        if name not in subtrees:
            raise self._error(lnr, f"Subtree {name!r} not defined")
        subtree = subtrees[name]
        specified = set(parameters) | set(references)
        available = set(subtree.parameter_list)
        if specified != available:
            raise self._error(
                lnr,
                f"Invalid parameters specified. Available parameters are {sorted(available)}, "
                f"specified parameters are {sorted(specified)}.",
            )
        return subtree, parameters, references

    def _instantiate_subtree(
        self, call: str, lnr: int, subtrees: dict[str, Tree[BlackboardT]]
    ) -> AbstractTreeElement[BlackboardT]:
        subtree, parameters, references = self._lookup_subtree(call, lnr, subtrees)
        if subtree.root_element is None:
            raise self._error(lnr, "Empty subtree")
        root = copy.deepcopy(subtree.root_element)
        root.parent = None
        self._evaluate_subtree_parameters(root, parameters, references, lnr)
        return root

    def _evaluate_subtree_parameters(
        self,
        root: AbstractTreeElement[BlackboardT],
        parameters: Parameters,
        unset_parameters: ParameterReferences,
        lnr: int,
    ) -> None:
        pending = [root]
        while pending:
            current = pending.pop()
            if isinstance(current, DecisionTreeElement):
                pending.extend(current.children.values())
            elif isinstance(current, ActionTreeElement) and current.next_element is not None:
                pending.append(current.next_element)
            for name, reference in list(current.unset_parameters.items()):
                if reference in parameters:
                    current.parameters[name] = parameters[reference]
                    del current.unset_parameters[name]
                elif reference in unset_parameters:
                    current.unset_parameters[name] = unset_parameters[reference]
                else:
                    raise self._error(lnr, f"Unknown reference to {reference!r}")
