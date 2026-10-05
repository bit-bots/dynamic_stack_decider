# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import annotations

import sys
from pathlib import Path
from typing import Protocol

import pydot
from ament_index_python import get_package_share_directory
from PyQt5.QtCore import Qt, QTimerEvent
from PyQt5.QtGui import QIcon, QPainter, QStandardItemModel
from PyQt5.QtSvg import QSvgGenerator
from PyQt5.QtWidgets import QCheckBox, QComboBox, QFileDialog, QGraphicsScene, QPushButton, QTreeView, QWidget
from PyQt5.uic import loadUi
from qt_dotgraph.dot_to_qt import DotToQtGenerator
from qt_dotgraph.pydotfactory import PydotFactory
from rclpy.node import Node
from rqt_gui.main import Main
from rqt_gui_py.plugin import Plugin

from .dsd_follower import DsdFollower
from .interactive_graphics_view import InteractiveGraphicsView


class PluginContext(Protocol):
    node: Node

    def add_widget(self, widget: QWidget) -> None: ...


class Settings(Protocol):
    def set_value(self, key: str, value: object) -> None: ...

    def value(self, key: str, default: object) -> object: ...


class DsdWidget(QWidget):
    """Widgets populated by the packaged Qt UI definition."""

    graphics_view: InteractiveGraphicsView
    stack_prop_tree_view: QTreeView
    dsd_selector_combo_box: QComboBox
    highlight_connections_check_box: QCheckBox
    show_full_tree: QCheckBox
    auto_fit_graph_check_box: QCheckBox
    fit_in_view_push_button: QPushButton
    freeze_push_button: QPushButton
    refresh_combobox_push_button: QPushButton
    save_as_svg_push_button: QPushButton


class DsdVizPlugin(Plugin):
    """Inspect a DSD discovered through its ROS 2 debug topics."""

    def __init__(self, context: PluginContext) -> None:
        super().__init__(context)
        self._node = context.node
        self._initialized = False
        self.freeze = False
        self.dsd_follower: DsdFollower | None = None
        self.running_dsd_instances: dict[str, str] = {}
        self._prev_dotgraph: pydot.Dot | None = None
        self._prev_item_model: QStandardItemModel | None = None
        self._messages: tuple[str, ...] | None = None
        self._timer_id: int | None = None
        self._init_plugin(context)

    def _init_plugin(self, context: PluginContext) -> None:
        self.setObjectName("DSD-Visualization")
        self._widget = DsdWidget()
        self._widget.setObjectName(self.objectName())
        ui_file = (
            Path(get_package_share_directory("dynamic_stack_decider_visualization")) / "resource/StackmachineViz.ui"
        )
        loadUi(str(ui_file), self._widget)
        self._scene = QGraphicsScene(self._widget)
        self._scene.setBackgroundBrush(Qt.white)
        self._widget.graphics_view.setScene(self._scene)
        self._widget.fit_in_view_push_button.setIcon(QIcon.fromTheme("view-fullscreen"))
        self._widget.fit_in_view_push_button.pressed.connect(self.fit_in_view)
        self._widget.auto_fit_graph_check_box.toggled.connect(lambda enabled: self.fit_in_view() if enabled else None)

        def toggle_freeze() -> None:
            self.freeze = self._widget.freeze_push_button.isChecked()
            self.refresh()

        def reset_dot_cache() -> None:
            if self.dsd_follower is not None:
                self.dsd_follower.reset_cache()

        self._widget.freeze_push_button.toggled.connect(toggle_freeze)
        self._widget.save_as_svg_push_button.pressed.connect(self.save_svg_to_file)
        self._widget.dsd_selector_combo_box.currentTextChanged.connect(self.set_dsd)
        self._widget.refresh_combobox_push_button.setIcon(QIcon.fromTheme("view-refresh"))
        self._widget.refresh_combobox_push_button.pressed.connect(self.discover_dsd_instances)
        self._widget.highlight_connections_check_box.toggled.connect(reset_dot_cache)
        self._widget.show_full_tree.toggled.connect(reset_dot_cache)
        self._widget.graphics_view.zoomed.connect(lambda: self._widget.auto_fit_graph_check_box.setChecked(False))
        self.discover_dsd_instances()
        context.add_widget(self._widget)
        self._timer_id = self.startTimer(100)

    def discover_dsd_instances(self) -> None:
        instances: dict[str, str] = {}
        for topic, types in self._node.get_topic_names_and_types():
            if topic.endswith("/dsd_tree") and "std_msgs/msg/String" in types:
                namespace = topic.removesuffix("/dsd_tree")
                instances[namespace] = namespace
        self.running_dsd_instances = instances
        selector = self._widget.dsd_selector_combo_box
        previous = selector.currentText()
        selector.blockSignals(True)
        try:
            selector.clear()
            selector.addItem("Select DSD...")
            selector.addItems(sorted(instances))
            if previous in instances:
                selector.setCurrentText(previous)
        finally:
            selector.blockSignals(False)
        self.set_dsd(selector.currentText())

    def save_settings(self, plugin_settings: object, instance_settings: Settings) -> None:
        super().save_settings(plugin_settings, instance_settings)
        instance_settings.set_value("auto_fit_graph_check_box_state", self._widget.auto_fit_graph_check_box.isChecked())
        instance_settings.set_value(
            "highlight_connections_check_box_state", self._widget.highlight_connections_check_box.isChecked()
        )
        instance_settings.set_value("show_full_tree_check_box_state", self._widget.show_full_tree.isChecked())

    def restore_settings(self, plugin_settings: object, instance_settings: Settings) -> None:
        super().restore_settings(plugin_settings, instance_settings)
        self._widget.auto_fit_graph_check_box.setChecked(
            instance_settings.value("auto_fit_graph_check_box_state", True) in (True, "true")
        )
        self._widget.highlight_connections_check_box.setChecked(
            instance_settings.value("highlight_connections_check_box_state", True) in (True, "true")
        )
        self._widget.show_full_tree.setChecked(
            instance_settings.value("show_full_tree_check_box_state", False) in (True, "true")
        )
        self._initialized = True
        self.refresh()

    def save_svg_to_file(self) -> None:
        file_name, _ = QFileDialog.getSaveFileName(
            self._widget, self.tr("Save as SVG"), "stackmachine.svg", self.tr("Scalable Vector Graphic (*.svg)")
        )
        if not file_name:
            return
        generator = QSvgGenerator()
        generator.setFileName(file_name)
        generator.setSize((self._scene.sceneRect().size() * 2.0).toSize())
        painter = QPainter(generator)
        try:
            painter.setRenderHint(QPainter.Antialiasing)
            self._scene.render(painter)
        finally:
            painter.end()

    def timerEvent(self, timer_event: QTimerEvent | None) -> None:
        if not self.freeze:
            self.refresh()
        if self._widget.auto_fit_graph_check_box.isChecked():
            self.fit_in_view()

    def fit_in_view(self) -> None:
        self._widget.graphics_view.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)

    def refresh(self) -> None:
        if not self._initialized:
            self._render_messages("The plugin is initializing.")
        elif self.dsd_follower is None:
            self._render_messages("No DSD selected")
        else:
            self._render_dotgraph(self.dsd_follower.to_dotgraph(self._widget.show_full_tree.isChecked()))
            self._render_debug_data(self.dsd_follower.to_q_item_model())

    def _render_messages(self, *messages: str) -> None:
        if messages == self._messages:
            return
        dot = pydot.Dot(graph_type="digraph")
        for index, message in enumerate(messages):
            dot.add_node(pydot.Node(f"message_{index}", label=message))
        self._render_dotgraph(dot)
        self._render_debug_data(QStandardItemModel(self._widget))
        self._messages = messages

    def _render_dotgraph(self, dotgraph: pydot.Dot) -> None:
        if self._prev_dotgraph is dotgraph:
            return
        self._prev_dotgraph = dotgraph
        self._messages = None
        self._scene.clear()
        highlight = 3 if self._widget.highlight_connections_check_box.isChecked() else 1
        nodes, edges = DotToQtGenerator().dotcode_to_qt_items(
            PydotFactory().create_dot(dotgraph), highlight, same_label_siblings=False
        )
        for node in nodes.values():
            self._scene.addItem(node)
        for group in edges.values():
            for edge in group:
                edge.add_to_scene(self._scene)
        self._scene.setSceneRect(self._scene.itemsBoundingRect())

    def _render_debug_data(self, model: QStandardItemModel) -> None:
        if self._prev_item_model is model:
            return
        self._prev_item_model = model
        self._widget.stack_prop_tree_view.setHeaderHidden(True)
        self._widget.stack_prop_tree_view.setModel(model)
        self._widget.stack_prop_tree_view.expandAll()

    def set_dsd(self, name: str) -> None:
        topic = self.running_dsd_instances.get(name)
        if self.dsd_follower is not None and self.dsd_follower.dsd_debug_topic == topic:
            return
        if self.dsd_follower is not None:
            self.dsd_follower.destroy()
        self.dsd_follower = DsdFollower(self._node, topic) if topic is not None else None
        self._messages = None

    def shutdown_plugin(self) -> None:
        if self._timer_id is not None:
            self.killTimer(self._timer_id)
            self._timer_id = None
        if self.dsd_follower is not None:
            self.dsd_follower.destroy()
            self.dsd_follower = None


def main() -> None:
    plugin = "dynamic_stack_decider_visualization.dsd_visualization_plugin.DsdVizPlugin"
    sys.exit(Main(filename=plugin).main(standalone=plugin))


if __name__ == "__main__":
    main()
