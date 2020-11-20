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
from __future__ import print_function

import os
import uuid

import pydot
import rospkg
import yaml
from .dsd_follower import DsdFollower
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QIcon, QPainter, QStandardItemModel
from python_qt_binding.QtWidgets import QFileDialog, QGraphicsScene, QWidget
from qt_dotgraph.dot_to_qt import DotToQtGenerator
from qt_dotgraph.pydotfactory import PydotFactory
from rqt_gui_py.plugin import Plugin

from .interactive_graphics_view import InteractiveGraphicsView


def parse_locations_yaml():
    rp = rospkg.RosPack()
    path = os.path.join(rp.get_path('dynamic_stack_decider_visualization'), 'config', 'locations.yaml')
    with open(path, 'r') as f:
        return yaml.load(f)['locations']


class DsdVizPlugin(Plugin):

    def __init__(self, context):
        super(DsdVizPlugin, self).__init__(context)

        self._initialized = False  # This gets set to true once the plugin hast completely finished loading

        # Ensure startup state
        self.freeze = False  # Controls whether the state should be updated from remote
        self.locations = parse_locations_yaml()
        self.dsd = None  # type: DsdFollower
        self._init_plugin(context)

        # Performance optimization variables
        self._prev_dotgraph = None
        self._prev_QItemModel = None

    def _init_plugin(self, context):
        self.setObjectName("DSD-Visualization")

        self._widget = QWidget()
        self._widget.setObjectName(self.objectName())

        # load qt ui definition from file
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path("dynamic_stack_decider_visualization"), "resource", "StackmachineViz.ui")
        loadUi(ui_file, self._widget, {"InteractiveGraphicsView": InteractiveGraphicsView})

        # initialize qt scene
        self._scene = QGraphicsScene()
        self._scene.setBackgroundBrush(Qt.white)
        self._widget.graphics_view.setScene(self._scene)

        # Bind fit-in-view button
        self._widget.fit_in_view_push_button.setIcon(QIcon.fromTheme('zoom-original'))
        self._widget.fit_in_view_push_button.pressed.connect(self.fit_in_view)

        # Fit-in-view on checkbox toggle
        self._widget.auto_fit_graph_check_box.toggled.connect(self.fit_in_view)

        # Freezing
        def toggle_freeze():
            self.freeze = not self.freeze
            self.refresh()

        self._widget.freeze_push_button.toggled.connect(toggle_freeze)

        # Exporting and importing
        self._widget.save_as_svg_push_button.pressed.connect(self.save_svg_to_file)

        # Fill choices for dsd_selector and bind on_select
        self._widget.dsd_selector_combo_box.addItem("Select DSD...")
        for choice in self.locations:
            self._widget.dsd_selector_combo_box.addItem(choice['display_name'])
        self._widget.dsd_selector_combo_box.currentTextChanged.connect(self.set_dsd)

        context.add_widget(self._widget)

        # Start a timer that calls back every 100 ms
        self._timer_id = self.startTimer(100)

    def save_settings(self, plugin_settings, instance_settings):
        super(DsdVizPlugin, self).save_settings(plugin_settings, instance_settings)

        instance_settings.set_value('auto_fit_graph_check_box_state', self._widget.auto_fit_graph_check_box.isChecked())
        instance_settings.set_value('highlight_connections_check_box_state',
                                    self._widget.highlight_connections_check_box.isChecked())

    def restore_settings(self, plugin_settings, instance_settings):
        super(DsdVizPlugin, self).restore_settings(plugin_settings, instance_settings)

        self._widget.auto_fit_graph_check_box.setChecked(
            instance_settings.value('auto_fit_graph_check_box_state', True) in [True, 'true'])
        self._widget.highlight_connections_check_box.setChecked(
            instance_settings.value('highlight_connections_check_box_state', True) in [True, 'true'])

        self._initialized = True
        self.refresh()

    def save_svg_to_file(self):
        file_name, _ = QFileDialog.getSaveFileName(self._widget, self.tr("Save as SVG"), "stackmachine.svg",
                                                   self.tr("Scalable Vector Graphic (*.svg)"))

        if file_name is not None and file_name != "":
            generator = QSvgGenerator()
            generator.setFileName(file_name)
            generator.setSize((self._scene.sceneRect().size() * 2.0).toSize())

            painter = QPainter(generator)
            painter.setRenderHint(QPainter.Antialiasing)
            self._scene.render(painter)
            painter.end()

    def timerEvent(self, timer_event):
        """This gets called by QT whenever the timer ticks"""

        if not self.freeze:
            self.refresh()

        # Automatically fit in view if the checkbox is checked
        if self._widget.auto_fit_graph_check_box.isChecked():
            self.fit_in_view()

    def fit_in_view(self):
        self._widget.graphics_view.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)

    def refresh(self):
        """Refresh the complete drawn representation"""

        if not self._initialized:
            self._render_messages("The plugin is not yet completely initialized. Please wait...")

        elif self.dsd is None:
            self._render_messages("No DSD selected")

        else:
            self._render_dotgraph(self.dsd.to_dotgraph())
            self._render_debug_data(self.dsd.to_QItemModel())

    def _render_messages(self, *messages):
        """Render simple messages on the canvas"""

        msg_dot = pydot.Dot()

        for msg in messages:
            msg_dot.add_node(pydot.Node(str(uuid.uuid4()), label=str(msg)))

        self._render_dotgraph(msg_dot)
        self._render_debug_data(QStandardItemModel(self._scene))

    def _render_dotgraph(self, dotgraph):
        """
        Render the specified dotgraph on canvas

        :type dotgraph: pydot.Dot
        """

        # Only redraw when the graph differs from the previous one
        if self._prev_dotgraph == dotgraph:
            return
        else:
            self._prev_dotgraph = dotgraph

        self._scene.clear()

        if self._widget.highlight_connections_check_box.isChecked():
            highlight_level = 3
        else:
            highlight_level = 1

        # Generate qt items from dotcode
        dotcode = PydotFactory().create_dot(dotgraph)
        nodes, edges = DotToQtGenerator().dotcode_to_qt_items(dotcode, highlight_level,
                                                              same_label_siblings=False)

        # Add generated items to scene
        for node_item in nodes:
            self._scene.addItem(nodes.get(node_item))
        for edge_items in edges:
            for edge_item in edges.get(edge_items):
                edge_item.add_to_scene(self._scene)

        self._scene.setSceneRect(self._scene.itemsBoundingRect())

    def _render_debug_data(self, qitem_model):
        """Render debug data in the tree view on the right side of the scene"""

        # Only redraw when the item-model differs from the previous one
        if self._prev_QItemModel == qitem_model:
            return
        else:
            self._prev_QItemModel = qitem_model
            self._widget.stack_prop_tree_view.setModel(qitem_model)
            self._widget.stack_prop_tree_view.expandAll()

    def set_dsd(self, name):
        """
        Set the target dsd

        :param name: display_name of any dsd in the locations.yaml
        """
        # close debug connection of old dsd
        if self.dsd is not None:
            self.dsd.close()

        if name == 'Select DSD...':
            self.dsd = None
            return

        # Search for dsd_data in locations.yaml
        for i in self.locations:
            if i['display_name'] == name:
                dsd_data = i
                break
        else:
            raise ValueError('no dsd with name {} found'.format(name))

        # Figure out full paths with the help of rospkg
        rospack = rospkg.RosPack()
        dsd_path = rospack.get_path(dsd_data['package'])
        actions_path = os.path.join(dsd_path, dsd_data['relative_action_path'])
        decisions_path = os.path.join(dsd_path, dsd_data['relative_decision_path'])
        behaviour_path = os.path.join(dsd_path, dsd_data['relative_dsd_path'])

        # Initialize dsd instance
        dsd = DsdFollower(dsd_data['debug_topic'])
        dsd.register_actions(actions_path)
        dsd.register_decisions(decisions_path)
        dsd.load_behavior(behaviour_path)
        dsd.initialized = True

        self.dsd = dsd
