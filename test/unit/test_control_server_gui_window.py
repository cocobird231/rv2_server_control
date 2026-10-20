"""Exercise Qt display, selection and operation guards without robot hardware."""

import time
from types import SimpleNamespace

import pytest
from PyQt5.QtCore import Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication
from r1_interfaces.msg import ControlServerStatus, ControlSignalInfo
from rv2_server_control_gui.window import ControlServerWindow


@pytest.fixture(scope="module")
def application():
    """Keep a Qt application alive for every widget in this module."""
    return QApplication.instance() or QApplication([])


@pytest.fixture
def window(application):
    """Supply a recording client; production ROS behavior has separate coverage."""
    state = SimpleNamespace(
        status=None,
        connected=False,
        connection_message="Waiting for status service",
        last_update=None,
        operation_pending=False,
        operation_message="",
        operation_success=None,
    )
    requests = []

    def record(operation, name=""):
        requests.append((operation, name))
        state.operation_pending = True
        state.operation_message = "Waiting for response"
        return True

    client = SimpleNamespace(
        state=state,
        target_prefix="/robot/control_server",
        switch_controller=lambda name: record("switch", name),
        reset_interrupt=lambda: record("reset"),
        requests=requests,
    )
    widget = ControlServerWindow(client)
    widget.show()
    application.processEvents()
    yield widget
    widget.close()
    widget.deleteLater()
    application.processEvents()


def snapshot(window, *, interrupted=False, names=("first", "second")):
    """Present an exact status snapshot, including zero-time publish metadata."""
    status = ControlServerStatus()
    status.server_name = "control_server"
    status.stamp.sec = 123
    status.interrupted = interrupted
    status.active_controller_name = names[0] if names else ""
    for name in names:
        info = ControlSignalInfo()
        info.controller_name = name
        info.channel_name = f"/input/{name}"
        info.type = "joy"
        info.priority = 10
        status.registered_controllers.append(info)
    state = window.client.state
    state.status = status
    state.connected = True
    state.last_update = time.monotonic()
    window.render()
    return status


def test_initial_state_has_no_actions_or_claimed_output(window):
    """Show unavailable data honestly until the first service reply."""
    assert "Waiting" in window.connection_label.text()
    assert "unavailable" in window.interrupt_label.text()
    assert window.active_label.text() == "Unknown"
    assert window.output_controller_label.text() == "Unknown"
    assert window.controller_table.rowCount() == 0
    assert not window.switch_button.isEnabled()
    assert not window.reset_button.isEnabled()


def test_real_button_click_sends_selected_identity_once(window):
    """Require explicit selection and suppress duplicate actions while pending."""
    snapshot(window)
    assert not window.switch_button.isEnabled()
    window.controller_table.selectRow(1)
    assert window.switch_button.isEnabled()
    QTest.mouseClick(window.switch_button, Qt.LeftButton)
    QTest.mouseClick(window.switch_button, Qt.LeftButton)
    assert window.client.requests == [("switch", "second")]
    assert not window.switch_button.isEnabled()
    assert not window.reset_button.isEnabled()


def test_interrupt_allows_only_explicit_reset(window):
    """Keep switching disabled while the authoritative interruption latch is set."""
    snapshot(window, interrupted=True)
    window.controller_table.selectRow(0)
    assert "latched" in window.interrupt_label.text()
    assert not window.switch_button.isEnabled()
    assert window.reset_button.isEnabled()
    QTest.mouseClick(window.reset_button, Qt.LeftButton)
    assert window.client.requests == [("reset", "")]
    assert window.client.state.status.interrupted
    assert not window.reset_button.isEnabled()


def test_stale_snapshot_retains_history_but_disables_actions(window):
    """Mark retained values stale instead of presenting a lost server as live."""
    snapshot(window, interrupted=True)
    window.controller_table.selectRow(1)
    window.client.state.connected = False
    window.client.state.connection_message = "Status request timed out"
    window.render()
    assert "timed out" in window.connection_label.text()
    assert "last snapshot" in window.interrupt_label.text()
    assert "STALE" in window.snapshot_label.text()
    assert window.active_label.text() == "first"
    assert not window.switch_button.isEnabled()
    assert not window.reset_button.isEnabled()


def test_selection_survives_refresh_and_clears_when_removed(window):
    """Preserve controller identity when registration order and contents change."""
    snapshot(window)
    window.controller_table.selectRow(1)
    snapshot(window, names=("second", "aaa", "first"))
    assert window.selected_controller() == "second"
    snapshot(window, names=("first",))
    assert window.selected_controller() == ""
    assert not window.switch_button.isEnabled()


def test_show_exact_last_output_without_substituting_current_owner(window):
    """Display zero ROS time, binary data and the actual previous output owner."""
    status = snapshot(window)
    status.has_output = True
    status.output_controller_name = "previous owner"
    status.output_api_id = 1008
    status.output_parameter = '{"x": 0.5, "y": 0.0, "z": 0.2}'
    status.output_binary = [0, 128, 255]
    window.render()
    assert window.active_label.text() == "first"
    assert window.output_controller_label.text() == "previous owner"
    assert window.output_stamp_label.text() == "0.000000000"
    assert window.api_label.text() == "1008 · Move"
    assert '"x": 0.5' in window.payload_text.toPlainText()
    assert window.binary_text.toPlainText() == "00 80 ff"
    status.output_controller_name = ""
    status.output_api_id = 98765
    status.output_parameter = "raw <data>"
    window.render()
    assert window.output_controller_label.text() == "Server / no controller"
    assert "98765" in window.api_label.text()
    assert window.payload_text.toPlainText() == "raw <data>"


def test_clear_old_request_after_server_restart(window):
    """Treat a fresh has_output=false snapshot as no publish in this incarnation."""
    status = snapshot(window)
    status.has_output = True
    status.output_parameter = "old request"
    window.render()
    snapshot(window, names=())
    assert window.output_controller_label.text() == "No output published"
    assert window.payload_text.toPlainText() == ""
    assert window.binary_text.toPlainText() == ""
    assert window.empty_label.isVisible()


def test_controller_names_and_errors_are_literal_text(window):
    """Prevent rich-text interpretation of externally supplied names and reasons."""
    snapshot(window, names=("<b>controller</b>",))
    window.client.state.operation_message = "Rejected: <b>not active</b>"
    window.client.state.operation_success = False
    window.render()
    assert window.active_label.textFormat() == Qt.PlainText
    assert window.active_label.text() == "<b>controller</b>"
    assert window.result_label.textFormat() == Qt.PlainText
    assert "<b>not active</b>" in window.result_label.text()
