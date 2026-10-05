"""Shared ROS subscription stand-ins and an offscreen Qt application."""

from unittest.mock import Mock

import pytest


class FakeNode:
    def __init__(self):
        self.logger = Mock()
        self.subscriptions = []
        self.destroyed = []
        self.topics = [("/robot/behavior/dsd_tree", ["std_msgs/msg/String"])]

    def get_logger(self):
        return self.logger

    def create_subscription(self, message_type, topic, callback, qos):
        handle = object()
        self.subscriptions.append((handle, topic, callback))
        return handle

    def destroy_subscription(self, subscription):
        self.destroyed.append(subscription)

    def get_topic_names_and_types(self):
        return self.topics


@pytest.fixture
def node():
    return FakeNode()


@pytest.fixture(scope="session")
def qapp():
    from PyQt5.QtWidgets import QApplication

    app = QApplication.instance() or QApplication([])
    yield app
    app.processEvents()
