#!/usr/bin/env python3

import math
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

try:
    from python_qt_binding.QtCore import QTimer, Qt
    from python_qt_binding.QtGui import QColor, QPainter, QPen
    from python_qt_binding.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QListWidget,
        QListWidgetItem,
        QMainWindow,
        QPushButton,
        QSplitter,
        QVBoxLayout,
        QWidget,
    )
except ImportError:
    from PyQt5.QtCore import QTimer, Qt
    from PyQt5.QtGui import QColor, QPainter, QPen
    from PyQt5.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QListWidget,
        QListWidgetItem,
        QMainWindow,
        QPushButton,
        QSplitter,
        QVBoxLayout,
        QWidget,
    )


def finite_number(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def is_numeric_scalar(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def ros_stamp_to_float(stamp):
    if stamp is None:
        return None
    if not hasattr(stamp, "sec") or not hasattr(stamp, "nanosec"):
        return None
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def message_stamp(msg):
    header = getattr(msg, "header", None)
    return ros_stamp_to_float(getattr(header, "stamp", None))


def extract_dynamic_joint_state(msg):
    if not hasattr(msg, "joint_names") or not hasattr(msg, "interface_values"):
        return {}

    values = {}
    for joint_index, joint_name in enumerate(msg.joint_names):
        if joint_index >= len(msg.interface_values):
            continue
        interface_values = msg.interface_values[joint_index]
        for interface_index, interface_name in enumerate(interface_values.interface_names):
            if interface_index >= len(interface_values.values):
                continue
            value = finite_number(interface_values.values[interface_index])
            if value is not None:
                values[f"{joint_name}/{interface_name}"] = value
    return values


def extract_joint_state(msg):
    if not hasattr(msg, "name"):
        return {}

    values = {}
    for field_name in ("position", "velocity", "effort"):
        field_values = getattr(msg, field_name, None)
        if field_values is None:
            continue
        for index, joint_name in enumerate(msg.name):
            if index >= len(field_values):
                continue
            value = finite_number(field_values[index])
            if value is not None:
                values[f"{joint_name}/{field_name}"] = value
    return values


def extract_numeric_fields(value, prefix=""):
    scalar = finite_number(value)
    if scalar is not None and prefix:
        return {prefix: scalar}

    if isinstance(value, (str, bytes, bytearray, bool)):
        return {}

    if isinstance(value, (list, tuple)):
        values = {}
        for index, item in enumerate(value):
            child_prefix = f"{prefix}[{index}]" if prefix else f"[{index}]"
            values.update(extract_numeric_fields(item, child_prefix))
        return values

    # ROS 2 array fields may arrive as array.array or numpy-like sequences.
    if hasattr(value, "__iter__") and not hasattr(value, "get_fields_and_field_types"):
        try:
            values = {}
            for index, item in enumerate(value):
                child_prefix = f"{prefix}[{index}]" if prefix else f"[{index}]"
                values.update(extract_numeric_fields(item, child_prefix))
            return values
        except TypeError:
            pass

    if hasattr(value, "get_fields_and_field_types"):
        values = {}
        for field_name in value.get_fields_and_field_types().keys():
            child = getattr(value, field_name)
            child_prefix = f"{prefix}.{field_name}" if prefix else field_name
            values.update(extract_numeric_fields(child, child_prefix))
        return values

    if is_numeric_scalar(value) and prefix:
        return {prefix: float(value)}

    return {}


def extract_values(msg):
    # Keep known aggregate messages readable, then fall back to generic recursion.
    dynamic_values = extract_dynamic_joint_state(msg)
    if dynamic_values:
        return dynamic_values

    joint_state_values = extract_joint_state(msg)
    if joint_state_values:
        return joint_state_values

    return extract_numeric_fields(msg)


class PlotCanvas(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(780, 460)
        self.series = {}
        self.enabled = set()
        self.window_seconds = 10.0
        self.time_source = "receive"
        self.y_mode = "shared"
        self.paused = False
        self.receive_t0 = time.monotonic()
        self.stamp_t0 = None
        self.colors = [
            QColor("#c62828"),
            QColor("#1565c0"),
            QColor("#2e7d32"),
            QColor("#ef6c00"),
            QColor("#6a1b9a"),
            QColor("#00838f"),
            QColor("#ad1457"),
            QColor("#37474f"),
            QColor("#9e9d24"),
            QColor("#5d4037"),
        ]

    def set_window_seconds(self, value):
        self.window_seconds = max(1.0, float(value))
        self.trim()
        self.update()

    def set_enabled(self, enabled):
        self.enabled = set(enabled)
        self.update()

    def set_time_source(self, value):
        self.time_source = value
        self.clear()

    def set_y_mode(self, value):
        self.y_mode = value
        self.update()

    def set_paused(self, paused):
        self.paused = paused

    def clear(self):
        self.series.clear()
        self.receive_t0 = time.monotonic()
        self.stamp_t0 = None
        self.update()

    def sample_time(self, msg_stamp):
        if self.time_source == "header" and msg_stamp is not None:
            if self.stamp_t0 is None:
                self.stamp_t0 = msg_stamp
            return msg_stamp - self.stamp_t0
        return time.monotonic() - self.receive_t0

    def add_sample(self, values, msg_stamp=None):
        if self.paused:
            return
        sample_time = self.sample_time(msg_stamp)
        for name, value in values.items():
            if name not in self.series:
                self.series[name] = deque()
            self.series[name].append((sample_time, value))
        self.trim(sample_time)
        self.update()

    def trim(self, now=None):
        if now is None:
            now = time.monotonic() - self.receive_t0
        min_time = now - self.window_seconds
        for points in self.series.values():
            while points and points[0][0] < min_time:
                points.popleft()

    def current_time_range(self):
        latest = 0.0
        for points in self.series.values():
            if points:
                latest = max(latest, points[-1][0])
        xmin = max(0.0, latest - self.window_seconds)
        xmax = max(self.window_seconds, latest)
        return xmin, xmax

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#ffffff"))
        painter.setRenderHint(QPainter.Antialiasing)

        left = 68
        right = 22
        top = 22
        bottom = 48
        width = max(1, self.width() - left - right)
        height = max(1, self.height() - top - bottom)
        plot_rect = (left, top, width, height)

        self.draw_grid(painter, plot_rect)

        names = [name for name in sorted(self.enabled) if name in self.series]
        if not names:
            painter.setPen(QColor("#555555"))
            painter.drawText(self.rect(), Qt.AlignCenter, "Select numeric fields to plot")
            painter.end()
            return

        xmin, xmax = self.current_time_range()
        if self.y_mode == "normalized":
            self.draw_normalized(painter, plot_rect, xmin, xmax, names)
            self.draw_axis_labels(painter, plot_rect, 0.0, 1.0, xmin, xmax, "normalized")
        elif self.y_mode == "per_series":
            self.draw_per_series(painter, plot_rect, xmin, xmax, names)
            self.draw_axis_labels(painter, plot_rect, 0.0, 1.0, xmin, xmax, "per series")
        else:
            ymin, ymax = self.shared_range(names)
            self.draw_shared(painter, plot_rect, xmin, xmax, ymin, ymax, names)
            self.draw_axis_labels(painter, plot_rect, ymin, ymax, xmin, xmax, "shared")

        self.draw_legend(painter, plot_rect, names)
        painter.end()

    def visible_points(self, name, xmin):
        return [(t, v) for t, v in self.series.get(name, []) if t >= xmin]

    def shared_range(self, names):
        values = []
        xmin, _ = self.current_time_range()
        for name in names:
            values.extend(v for _, v in self.visible_points(name, xmin))
        if not values:
            return -1.0, 1.0
        ymin = min(values)
        ymax = max(values)
        if math.isclose(ymin, ymax):
            ymin -= 1.0
            ymax += 1.0
        pad = 0.08 * (ymax - ymin)
        return ymin - pad, ymax + pad

    def series_range(self, name, xmin):
        values = [v for _, v in self.visible_points(name, xmin)]
        if not values:
            return -1.0, 1.0
        ymin = min(values)
        ymax = max(values)
        if math.isclose(ymin, ymax):
            ymin -= 1.0
            ymax += 1.0
        return ymin, ymax

    def map_point(self, t, v, rect, xmin, xmax, ymin, ymax):
        left, top, width, height = rect
        x = left + int((t - xmin) / max(1e-9, xmax - xmin) * width)
        y = top + height - int((v - ymin) / max(1e-9, ymax - ymin) * height)
        return x, y

    def draw_series(self, painter, rect, xmin, xmax, ymin, ymax, name, color):
        painter.setPen(QPen(color, 2))
        last_xy = None
        for t, value in self.visible_points(name, xmin):
            xy = self.map_point(t, value, rect, xmin, xmax, ymin, ymax)
            if last_xy is not None:
                painter.drawLine(last_xy[0], last_xy[1], xy[0], xy[1])
            last_xy = xy

    def draw_shared(self, painter, rect, xmin, xmax, ymin, ymax, names):
        for index, name in enumerate(names):
            self.draw_series(painter, rect, xmin, xmax, ymin, ymax, name, self.colors[index % len(self.colors)])

    def draw_normalized(self, painter, rect, xmin, xmax, names):
        for index, name in enumerate(names):
            ymin, ymax = self.series_range(name, xmin)
            color = self.colors[index % len(self.colors)]
            painter.setPen(QPen(color, 2))
            last_xy = None
            for t, value in self.visible_points(name, xmin):
                norm = (value - ymin) / max(1e-9, ymax - ymin)
                xy = self.map_point(t, norm, rect, xmin, xmax, 0.0, 1.0)
                if last_xy is not None:
                    painter.drawLine(last_xy[0], last_xy[1], xy[0], xy[1])
                last_xy = xy

    def draw_per_series(self, painter, rect, xmin, xmax, names):
        if not names:
            return
        left, top, width, height = rect
        band_height = max(1, height // len(names))
        for index, name in enumerate(names):
            band_top = top + index * band_height
            band_rect = (left, band_top, width, band_height)
            ymin, ymax = self.series_range(name, xmin)
            self.draw_series(painter, band_rect, xmin, xmax, ymin, ymax, name, self.colors[index % len(self.colors)])
            painter.setPen(QPen(QColor("#e0e0e0"), 1))
            painter.drawLine(left, band_top, left + width, band_top)

    def draw_grid(self, painter, rect):
        left, top, width, height = rect
        painter.setPen(QPen(QColor("#d0d0d0"), 1))
        painter.drawRect(left, top, width, height)
        for i in range(1, 6):
            x = left + int(width * i / 6)
            painter.drawLine(x, top, x, top + height)
        for i in range(1, 4):
            y = top + int(height * i / 4)
            painter.drawLine(left, y, left + width, y)

    def draw_axis_labels(self, painter, rect, ymin, ymax, xmin, xmax, mode):
        left, top, width, height = rect
        painter.setPen(QColor("#333333"))
        painter.drawText(6, top + 12, f"{ymax:.4g}")
        painter.drawText(6, top + height, f"{ymin:.4g}")
        painter.drawText(left, top + height + 28, f"{xmin:.2f}s")
        painter.drawText(left + width - 72, top + height + 28, f"{xmax:.2f}s")
        painter.drawText(left + width // 2 - 40, top + height + 28, mode)

    def draw_legend(self, painter, rect, names):
        left, top, _, _ = rect
        legend_x = left + 10
        legend_y = top + 18
        max_items = min(10, len(names))
        for index, name in enumerate(names[:max_items]):
            color = self.colors[index % len(self.colors)]
            painter.setPen(QPen(color, 3))
            y = legend_y + index * 18
            painter.drawLine(legend_x, y - 5, legend_x + 18, y - 5)
            painter.setPen(QColor("#222222"))
            painter.drawText(legend_x + 24, y, name)
        if len(names) > max_items:
            painter.setPen(QColor("#666666"))
            painter.drawText(legend_x + 24, legend_y + max_items * 18, f"+ {len(names) - max_items} more")


class TopicPlotterWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.subscription = None
        self.available_fields = set()

        self.setWindowTitle("ROS Topic Plotter")
        self.resize(1260, 760)

        self.topic_combo = QComboBox()
        self.topic_combo.setMinimumWidth(380)
        self.type_label = QLabel("type: -")
        self.refresh_button = QPushButton("Refresh")
        self.subscribe_button = QPushButton("Subscribe")
        self.filter_edit = QLineEdit()
        self.filter_edit.setPlaceholderText("Filter numeric fields")
        self.select_all_box = QCheckBox("Select all")
        self.clear_button = QPushButton("Clear")
        self.pause_box = QCheckBox("Pause")

        self.window_spin = QDoubleSpinBox()
        self.window_spin.setRange(1.0, 3600.0)
        self.window_spin.setValue(10.0)
        self.window_spin.setSuffix(" s")

        self.time_combo = QComboBox()
        self.time_combo.addItem("Receive time", "receive")
        self.time_combo.addItem("Header stamp", "header")

        self.y_combo = QComboBox()
        self.y_combo.addItem("Shared Y", "shared")
        self.y_combo.addItem("Normalized", "normalized")
        self.y_combo.addItem("Per series", "per_series")

        self.field_list = QListWidget()
        self.canvas = PlotCanvas()

        controls = QHBoxLayout()
        controls.addWidget(QLabel("Topic"))
        controls.addWidget(self.topic_combo)
        controls.addWidget(self.refresh_button)
        controls.addWidget(self.subscribe_button)
        controls.addWidget(self.type_label)
        controls.addStretch()
        controls.addWidget(QLabel("Time"))
        controls.addWidget(self.time_combo)
        controls.addWidget(QLabel("Y"))
        controls.addWidget(self.y_combo)
        controls.addWidget(QLabel("Window"))
        controls.addWidget(self.window_spin)
        controls.addWidget(self.pause_box)
        controls.addWidget(self.clear_button)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.addWidget(self.filter_edit)
        left_layout.addWidget(self.select_all_box)
        left_layout.addWidget(self.field_list)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(self.canvas)
        splitter.setSizes([380, 880])

        root = QWidget()
        root_layout = QVBoxLayout(root)
        root_layout.addLayout(controls)
        root_layout.addWidget(splitter)
        self.setCentralWidget(root)

        self.refresh_button.clicked.connect(self.refresh_topics)
        self.subscribe_button.clicked.connect(self.subscribe_selected_topic)
        self.filter_edit.textChanged.connect(self.rebuild_field_list)
        self.select_all_box.stateChanged.connect(self.set_all_visible_fields)
        self.clear_button.clicked.connect(self.clear_plot)
        self.pause_box.stateChanged.connect(lambda state: self.canvas.set_paused(state == Qt.Checked))
        self.window_spin.valueChanged.connect(self.canvas.set_window_seconds)
        self.time_combo.currentIndexChanged.connect(lambda _index: self.canvas.set_time_source(self.time_combo.currentData()))
        self.y_combo.currentIndexChanged.connect(lambda _index: self.canvas.set_y_mode(self.y_combo.currentData()))
        self.field_list.itemChanged.connect(self.update_enabled_fields)
        self.topic_combo.currentIndexChanged.connect(lambda _index: self.update_type_label())

        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self.spin_ros)
        self.spin_timer.start(10)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh_topics)
        self.refresh_timer.start(3000)

        self.refresh_topics()

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def refresh_topics(self):
        current = self.topic_combo.currentText()
        topics = sorted(self.node.get_topic_names_and_types())
        self.topic_combo.blockSignals(True)
        self.topic_combo.clear()
        for topic_name, topic_types in topics:
            if topic_types:
                self.topic_combo.addItem(topic_name, topic_types[0])
        index = self.topic_combo.findText(current)
        if index >= 0:
            self.topic_combo.setCurrentIndex(index)
        self.topic_combo.blockSignals(False)
        self.update_type_label()

    def update_type_label(self):
        topic_type = self.topic_combo.currentData()
        self.type_label.setText(f"type: {topic_type or '-'}")

    def subscribe_selected_topic(self):
        topic_name = self.topic_combo.currentText()
        topic_type = self.topic_combo.currentData()
        if not topic_name or not topic_type:
            return

        if self.subscription is not None:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None

        try:
            msg_class = get_message(topic_type)
        except (AttributeError, ModuleNotFoundError, ValueError) as exc:
            self.node.get_logger().error(f"Cannot load message type {topic_type}: {exc}")
            return

        self.available_fields.clear()
        self.canvas.clear()
        self.rebuild_field_list()

        self.subscription = self.node.create_subscription(msg_class, topic_name, self.on_message, 10)
        self.node.get_logger().info(f"Subscribed to {topic_name} [{topic_type}]")

    def on_message(self, msg):
        values = extract_values(msg)
        if not values:
            return

        new_fields = set(values.keys()) - self.available_fields
        self.available_fields.update(values.keys())
        if new_fields:
            self.rebuild_field_list()

        self.canvas.add_sample(values, message_stamp(msg))

    def visible_field_names(self):
        filter_text = self.filter_edit.text().strip().lower()
        return [
            field
            for field in sorted(self.available_fields)
            if not filter_text or filter_text in field.lower()
        ]

    def rebuild_field_list(self):
        checked = self.checked_fields()
        self.field_list.blockSignals(True)
        self.field_list.clear()
        for field_name in self.visible_field_names():
            item = QListWidgetItem(field_name)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Checked if field_name in checked else Qt.Unchecked)
            self.field_list.addItem(item)
        self.field_list.blockSignals(False)
        self.update_select_all_state()
        self.update_enabled_fields()

    def checked_fields(self):
        fields = set()
        for index in range(self.field_list.count()):
            item = self.field_list.item(index)
            if item.checkState() == Qt.Checked:
                fields.add(item.text())
        return fields

    def set_all_visible_fields(self, state):
        self.field_list.blockSignals(True)
        for index in range(self.field_list.count()):
            self.field_list.item(index).setCheckState(Qt.Checked if state == Qt.Checked else Qt.Unchecked)
        self.field_list.blockSignals(False)
        self.update_enabled_fields()

    def update_select_all_state(self):
        total = self.field_list.count()
        checked = len(self.checked_fields())
        self.select_all_box.blockSignals(True)
        if total > 0 and checked == total:
            self.select_all_box.setCheckState(Qt.Checked)
        elif checked == 0:
            self.select_all_box.setCheckState(Qt.Unchecked)
        else:
            self.select_all_box.setCheckState(Qt.PartiallyChecked)
        self.select_all_box.blockSignals(False)

    def update_enabled_fields(self):
        self.update_select_all_state()
        self.canvas.set_enabled(self.checked_fields())

    def clear_plot(self):
        self.canvas.clear()


def main():
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    node = Node("ros_topic_plotter")
    window = TopicPlotterWindow(node)
    window.show()
    exit_code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
