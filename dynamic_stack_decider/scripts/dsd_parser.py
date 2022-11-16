#!/usr/bin/env python3

import json
import os
import sys
from types import SimpleNamespace

from dynamic_stack_decider import DSD
from dynamic_stack_decider.abstract_stack_element import AbstractStackElement
from dynamic_stack_decider.tree import (AbstractTreeElement, ActionTreeElement,
                                        DecisionTreeElement,
                                        SequenceTreeElement)


class DsdJsonConverter():
    def __init__(self, dsd_file_path: str) -> None:
        self.dsd_file_path = os.path.abspath(dsd_file_path)
        self.dsd_folder = os.path.abspath(os.path.dirname(dsd_file_path))
        self.actions_folder = self.dsd_folder + '/actions'
        self.decisions_folder = self.dsd_folder + '/decisions'

    def setup_dsd(self) -> DSD:
        node = SimpleNamespace()
        def fake_get_parameter_fn(key): return SimpleNamespace(
            value="node_param." + key)
        node.get_parameter = fake_get_parameter_fn

        dsd = DSD(None, node=node)
        dsd.register_actions(self.actions_folder)
        dsd.register_decisions(self.decisions_folder)

        return dsd

    def parse(self) -> str:
        dsd = self.setup_dsd()
        dsd.load_behavior(self.dsd_file_path)
        dsd_tree = self.to_json(dsd.tree.root_element)
        return json.dumps(dsd_tree)

    def to_json(self, element) -> dict:
        conversion_object = dict()
        if (isinstance(element, tuple)):
            conversion_object['option'] = element[0]
            conversion_object['result'] = self.to_json(element[1])

        if (isinstance(element, AbstractTreeElement)):
            if (element.name):
                conversion_object['name'] = element.name

        if (isinstance(element, DecisionTreeElement)):
            conversion_object['type'] = 'decision'
            conversion_object['path'] = self.extract_folder(
                self.decisions_folder, element.module)
            conversion_object['parameters'] = element.parameters
            if (element.unset_parameters):
                conversion_object['unset_parameters'] = element.unset_parameters
            conversion_object['options'] = list(
                map(self.to_json, element.children.items()))

        if (isinstance(element, SequenceTreeElement)):
            conversion_object['type'] = 'sequence'
            conversion_object['action_sequence'] = list(
                map(self.to_json, element.action_elements))

        if (isinstance(element, ActionTreeElement)):
            conversion_object['type'] = 'action'
            conversion_object['path'] = self.extract_folder(
                self.actions_folder, element.module)
            conversion_object['parameters'] = element.parameters
            if (element.unset_parameters):
                conversion_object['unset_parameters'] = element.unset_parameters

        return conversion_object

    def extract_folder(self, type_folder: str, module: AbstractStackElement) -> str:
        return os.path.abspath(f"{type_folder}/{module.__module__.replace('.', '/')}")


if __name__ == "__main__":
    result = DsdJsonConverter(sys.argv[1]).parse()
    print(result)
    sys.exit()
