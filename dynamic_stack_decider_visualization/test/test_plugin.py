"""Load the real Qt UI and Graphviz renderer without a running robot."""

import importlib
import os
from pathlib import Path

import pytest

if os.environ.get("DSD_REQUIRE_GUI"):
    importlib.import_module("PyQt5.QtWidgets")
else:
    pytest.importorskip("PyQt5.QtWidgets")

import dynamic_stack_decider_visualization.dsd_visualization_plugin as plugin_module
from PyQt5.QtCore import QObject, QPoint, QPointF, Qt
from PyQt5.QtGui import QWheelEvent


class Context(QObject):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.widgets = []

    def add_widget(self, widget):
        self.widgets.append(widget)


class Settings:
    def value(self, key, default):
        return default


@pytest.fixture
def plugin(qapp, node, monkeypatch):
    package = Path(__file__).resolve().parents[1]
    monkeypatch.setattr(plugin_module, "get_package_share_directory", lambda name: str(package))
    context = Context(node)
    instance = plugin_module.DsdVizPlugin(context)
    instance.restore_settings(object(), Settings())
    yield instance
    instance.shutdown_plugin()
    for widget in context.widgets:
        widget.close()
    qapp.processEvents()


def test_ui_loads_and_waiting_message_renders(plugin):
    assert plugin._scene.items()
    model = plugin._prev_item_model
    graph = plugin._prev_dotgraph
    plugin.refresh()
    assert plugin._prev_item_model is model
    assert plugin._prev_dotgraph is graph


def test_deselecting_releases_subscriptions(plugin, node):
    plugin.set_dsd("/robot/behavior")
    assert plugin.dsd_follower is not None
    plugin.set_dsd("Select DSD...")
    assert plugin.dsd_follower is None
    assert len(node.destroyed) == 2


def test_refresh_preserves_selection_without_resubscribing(plugin, node):
    plugin._widget.dsd_selector_combo_box.setCurrentText("/robot/behavior")
    original = plugin.dsd_follower
    plugin.discover_dsd_instances()
    assert plugin.dsd_follower is original
    assert len(node.subscriptions) == 2
    assert node.destroyed == []


def test_disappearing_topic_releases_subscriptions(plugin, node):
    plugin._widget.dsd_selector_combo_box.setCurrentText("/robot/behavior")
    node.topics = []
    plugin.discover_dsd_instances()
    assert plugin.dsd_follower is None
    assert len(node.destroyed) == 2


def test_shutdown_stops_timer_and_releases_subscriptions(plugin, node):
    plugin.set_dsd("/robot/behavior")
    plugin.shutdown_plugin()
    plugin.shutdown_plugin()
    assert plugin._timer_id is None
    assert plugin.dsd_follower is None
    assert len(node.destroyed) == 2


def test_zoom_signal_disables_auto_fit(plugin):
    view = plugin._widget.graphics_view
    plugin._widget.auto_fit_graph_check_box.setChecked(True)
    before = view.transform().m11()
    event = QWheelEvent(
        QPointF(10, 10),
        QPointF(10, 10),
        QPoint(0, 0),
        QPoint(0, 120),
        Qt.NoButton,
        Qt.NoModifier,
        Qt.NoScrollPhase,
        False,
    )
    view.wheelEvent(event)
    assert not plugin._widget.auto_fit_graph_check_box.isChecked()
    assert view.transform().m11() > before


def test_live_tree_and_stack_render_in_the_actual_scene(plugin):
    import json

    from std_msgs.msg import String

    plugin.set_dsd("/robot/behavior")
    follower = plugin.dsd_follower
    tree = {
        "id": 1,
        "type": "decision",
        "name": "Mode",
        "parameters": {},
        "children": {"GO": {"id": 2, "type": "action", "name": "Idle", "parameters": {}, "next": None}},
    }
    stack = {
        "id": 1,
        "type": "decision",
        "name": "Mode",
        "debug_data": {},
        "activation_reason": None,
        "next": {
            "id": 2,
            "type": "action",
            "name": "Idle",
            "debug_data": {"status": "active"},
            "activation_reason": "GO",
            "next": None,
        },
    }
    follower._tree_callback(String(data=json.dumps(tree)))
    follower._stack_callback(String(data=json.dumps(stack)))
    plugin.refresh()
    assert len(plugin._scene.items()) > 2
    assert plugin._prev_item_model.item(2).child(0).text() == "status: active"


def test_svg_export_writes_rendered_scene(plugin, tmp_path, monkeypatch):
    path = tmp_path / "scene.svg"
    monkeypatch.setattr(plugin_module.QFileDialog, "getSaveFileName", lambda *args: (str(path), ""))
    plugin.save_svg_to_file()
    assert "<svg" in path.read_text()


def test_freeze_prevents_timer_refresh(plugin, monkeypatch):
    from unittest.mock import Mock

    refresh = Mock()
    monkeypatch.setattr(plugin, "refresh", refresh)
    plugin.freeze = True
    plugin.timerEvent(None)
    refresh.assert_not_called()
    plugin.freeze = False
    plugin.timerEvent(None)
    refresh.assert_called_once()
