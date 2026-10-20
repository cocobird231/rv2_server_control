"""Present server snapshots and explicit controller operations with Qt widgets."""

import json
import time

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

API_NAMES = {
    1003: "StopMove",
    1004: "StandUp",
    1005: "StandDown",
    1006: "RecoveryStand",
    1008: "Move",
}


def timestamp_text(stamp):
    """Show the full ROS timestamp, including a valid zero-time output."""
    return f"{stamp.sec}.{stamp.nanosec:09d}"


def plain_label(text=""):
    """Keep controller names and service diagnostics literal and selectable."""
    label = QLabel(text)
    label.setTextFormat(Qt.PlainText)
    label.setWordWrap(True)
    label.setMinimumHeight(label.fontMetrics().height())
    label.setTextInteractionFlags(Qt.TextSelectableByMouse)
    return label


class ControlServerWindow(QWidget):
    """Display one client's state without publishing robot control messages."""

    def __init__(self, client):
        super().__init__()
        self.client = client
        self._rows = None
        self.setWindowTitle(f"Control server — {client.target_prefix}")
        self.resize(980, 860)
        self.setMinimumSize(660, 600)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        outer.addWidget(scroll)
        content = QWidget()
        scroll.setWidget(content)
        layout = QVBoxLayout(content)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(14)

        title = plain_label("Control server")
        font = title.font()
        font.setPointSize(19)
        font.setBold(True)
        title.setFont(font)
        self.connection_badge = plain_label()
        self.connection_label = plain_label()
        header = QHBoxLayout()
        header.addWidget(title, 1)
        header.addWidget(self.connection_badge, 1, Qt.AlignRight)
        layout.addLayout(header)
        self.target_label = plain_label(f"Service target: {client.target_prefix}")
        layout.addWidget(self.target_label)
        layout.addWidget(self.connection_label)

        self.interrupt_label = plain_label()
        self.interrupt_label.setContentsMargins(12, 9, 12, 9)
        layout.addWidget(self.interrupt_label)

        current = QFormLayout()
        self.server_label = plain_label()
        self.active_label = plain_label()
        self.snapshot_label = plain_label()
        current.addRow("Server name", self.server_label)
        current.addRow("Current controller", self.active_label)
        current.addRow("Snapshot / freshness", self.snapshot_label)
        layout.addLayout(current)

        controllers = QGroupBox("Registered controllers")
        controller_layout = QVBoxLayout(controllers)
        self.controller_table = QTableWidget(0, 5)
        self.controller_table.setHorizontalHeaderLabels(
            ["Controller", "Type", "Priority", "Channel", "Output"]
        )
        self.controller_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.controller_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.controller_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.controller_table.verticalHeader().hide()
        self.controller_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeToContents
        )
        self.controller_table.horizontalHeader().setSectionResizeMode(
            3, QHeaderView.Stretch
        )
        self.controller_table.setMinimumHeight(130)
        self.controller_table.setMaximumHeight(150)
        self.controller_table.setAccessibleName("Registered controllers")
        self.controller_table.itemSelectionChanged.connect(self._update_actions)
        controller_layout.addWidget(self.controller_table)
        self.empty_label = plain_label("No registered controllers.")
        controller_layout.addWidget(self.empty_label)
        controller_layout.addWidget(
            plain_label("Select a row to switch. The server checks ACTIVE state.")
        )
        operations = QHBoxLayout()
        self.switch_button = QPushButton("Switch to selected controller")
        self.switch_button.clicked.connect(self._switch)
        self.reset_button = QPushButton("Reset wireless interrupt")
        self.reset_button.setToolTip(
            "Clear the latch only. New controller input is required to resume."
        )
        self.reset_button.clicked.connect(self._reset)
        operations.addWidget(self.switch_button)
        operations.addStretch()
        operations.addWidget(self.reset_button)
        controller_layout.addLayout(operations)
        self.result_label = plain_label()
        controller_layout.addWidget(self.result_label)
        layout.addWidget(controllers)

        output = QGroupBox("Last output published by this server")
        output_layout = QVBoxLayout(output)
        details = QFormLayout()
        self.output_controller_label = plain_label()
        self.api_label = plain_label()
        self.output_stamp_label = plain_label()
        details.addRow("Output controller", self.output_controller_label)
        details.addRow("API", self.api_label)
        details.addRow("Published at (ROS time)", self.output_stamp_label)
        output_layout.addLayout(details)
        output_layout.addWidget(plain_label("Parameter"))
        self.payload_text = QPlainTextEdit()
        self.payload_text.setReadOnly(True)
        self.payload_text.setAccessibleName("Last output parameter")
        self.payload_text.setMinimumHeight(90)
        self.payload_text.setMaximumHeight(90)
        output_layout.addWidget(self.payload_text, 1)
        output_layout.addWidget(plain_label("Binary (hex)"))
        self.binary_text = QPlainTextEdit()
        self.binary_text.setReadOnly(True)
        self.binary_text.setAccessibleName("Last output binary")
        self.binary_text.setMaximumHeight(64)
        self.binary_text.setMinimumHeight(48)
        output_layout.addWidget(self.binary_text)
        output_layout.addWidget(
            plain_label("Local publish history; this is not a robot acknowledgement.")
        )
        layout.addWidget(output)
        layout.addStretch()
        self.render()

    def selected_controller(self):
        """Return the controller identity, never its topic or display decoration."""
        rows = self.controller_table.selectionModel().selectedRows()
        return self.controller_table.item(rows[0].row(), 0).text() if rows else ""

    def _update_actions(self):
        state = self.client.state
        ready = state.connected and not state.operation_pending and state.status
        self.switch_button.setEnabled(
            bool(ready and not state.status.interrupted and self.selected_controller())
        )
        self.reset_button.setEnabled(bool(ready and state.status.interrupted))

    def _switch(self):
        self.client.switch_controller(self.selected_controller())
        self.render()

    def _reset(self):
        self.client.reset_interrupt()
        self.render()

    def _render_controllers(self, status):
        rows = (
            tuple(
                sorted(
                    (
                        info.controller_name,
                        info.type,
                        str(info.priority),
                        info.channel_name,
                        "Current"
                        if info.controller_name == status.active_controller_name
                        else "",
                    )
                    for info in status.registered_controllers
                )
            )
            if status
            else ()
        )
        if rows == self._rows:
            return
        selected = self.selected_controller()
        self.controller_table.blockSignals(True)
        self.controller_table.setRowCount(0)
        self.controller_table.setRowCount(len(rows))
        for row, values in enumerate(rows):
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setToolTip(value)
                self.controller_table.setItem(row, column, item)
            if values[0] == selected:
                self.controller_table.selectRow(row)
        self.controller_table.blockSignals(False)
        self.empty_label.setVisible(not rows)
        self._rows = rows

    @staticmethod
    def _set_payload(widget, text):
        # Preserve selection and scrolling while an unchanged snapshot is polled.
        if widget.toPlainText() != text:
            widget.setPlainText(text)

    def render(self):
        """Refresh visible facts, retaining old data only with an explicit stale label."""
        state = self.client.state
        status = state.status
        self.connection_label.setText(
            "Connected" if state.connected else state.connection_message
        )
        self.connection_label.setVisible(not state.connected)
        self.connection_badge.setText("Connected" if state.connected else "Unavailable")
        self.connection_badge.setStyleSheet(
            "color: #17613a;" if state.connected else "color: #6b3b14;"
        )
        interrupted = bool(status and status.interrupted)
        if not state.connected:
            banner = "Status unavailable — controls disabled."
            if status:
                banner += " Showing the last snapshot."
            color, background = "#454b53", "#edf0f3"
        elif interrupted:
            banner = "Wireless interrupt is latched — manual reset required."
            color, background = "#7a3d00", "#fff0d6"
        else:
            banner = "Wireless interrupt is clear."
            color, background = "#175239", "#e6f3ec"
        self.interrupt_label.setText(banner)
        self.interrupt_label.setStyleSheet(
            f"color: {color}; background-color: {background};"
        )
        self.active_label.setText(
            (status.active_controller_name or "None selected") if status else "Unknown"
        )
        self.server_label.setText(status.server_name if status else "Unknown")
        if status:
            age = (
                f"{max(0.0, time.monotonic() - state.last_update):.1f}s ago"
                if state.last_update is not None
                else "not confirmed"
            )
            self.snapshot_label.setText(
                f"{timestamp_text(status.stamp)} · {age}"
                + (" · STALE" if not state.connected else "")
            )
        else:
            self.snapshot_label.setText("Waiting for the first status response")
        self._render_controllers(status)
        if status and status.has_output:
            self.output_controller_label.setText(
                status.output_controller_name or "Server / no controller"
            )
            name = API_NAMES.get(status.output_api_id, "Unknown API")
            self.api_label.setText(f"{status.output_api_id} · {name}")
            self.output_stamp_label.setText(timestamp_text(status.output_stamp))
            payload = status.output_parameter
            try:
                payload = json.dumps(json.loads(payload), indent=2, ensure_ascii=False)
            except (ValueError, TypeError):
                pass
            self._set_payload(self.payload_text, payload)
            self._set_payload(self.binary_text, bytes(status.output_binary).hex(" "))
        else:
            text = "No output published" if status else "Unknown"
            self.output_controller_label.setText(text)
            self.api_label.setText("—")
            self.output_stamp_label.setText("—")
            self._set_payload(self.payload_text, "")
            self._set_payload(self.binary_text, "")
        self.result_label.setText(state.operation_message)
        self.result_label.setVisible(bool(state.operation_message))
        self.result_label.setStyleSheet(
            "color: #9b2525;" if state.operation_success is False else "color: #31536c;"
        )
        self._update_actions()
