from PyQt5.QtWidgets import QGraphicsItem, QGraphicsScene

class EdgeItem:
    def add_to_scene(self, scene: QGraphicsScene) -> None: ...

class DotToQtGenerator:
    def dotcode_to_qt_items(
        self, dotcode: str, highlight_level: int, *, same_label_siblings: bool
    ) -> tuple[dict[str, QGraphicsItem], dict[str, list[EdgeItem]]]: ...
