# External typing boundaries

These stubs describe the Qt/RQT APIs used by the visualization. The installed
ROS packages do not ship typing metadata. Keep signatures aligned with the
RoboStack versions selected in `pixi.lock`. Qt itself uses the `pyqt-stubs`
package and pydot supplies its own annotations.

ROS client/message imports and `PyQt5.uic` are isolated as untyped third-party
boundaries in the mypy configuration. Library code is checked in strict mode;
there are no per-line type suppressions.
