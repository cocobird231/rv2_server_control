"""Exercise the real GUI against production R1 peers and observable sport output."""

import json
import os
import signal
import sys
import threading
import time
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, SetLaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_testing import post_shutdown_test
from launch_testing.actions import ReadyToTest
from launch_testing.asserts import assertExitCodes
from PyQt5.QtCore import Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication
from r1_interfaces.msg import EntryStatus, ManagerStatus
from rclpy import create_node, init, shutdown
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rv2_server_control_gui.client import ControlServerGuiNode
from rv2_server_control_gui.window import ControlServerWindow
from sensor_msgs.msg import Joy
from unitree_api.msg import Request
from unitree_go.msg import WirelessController

SERVER, MASTER = "r1_gui_server", "r1_gui_master"
HIGH, LOW = "gui_high_priority", "gui_low_priority"
MOVE, STOP = 1008, 1003
PRIORITIES = {HIGH: 100, LOW: 10}
SPEEDS = {HIGH: 0.2, LOW: 0.4}


def generate_test_description():
    """Use two genuine bridges and leave all robot hardware absent."""
    master = Node(
        package="rv2_control_signal_transport",
        executable="csm_master_node",
        parameters=[{"master_name": MASTER, "tick_interval_ms": 50}],
        output="screen",
    )
    server = Node(
        package="rv2_server_control",
        executable="control_server",
        parameters=[
            {
                "server_name": SERVER,
                "master_name": MASTER,
                "status_timer_interval_ms": 50,
                "watchdog_interval_ms": 20,
            }
        ],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )
    bridges = [
        Node(
            package="rv2_csm_topic_bridge",
            executable="topic_bridge",
            name=f"{name}_bridge",
            parameters=[
                {
                    "topic_name": f"/{SERVER}/{name}/joy",
                    "server_name": SERVER,
                    "master_name": MASTER,
                    "csm_name": f"{name}_manager",
                    "controller_name": name,
                    "channel_name": f"/{SERVER}/{name}/control",
                    "priority": priority,
                    "timeout_ms": 1500,
                    "disconnect_timeout_ms": 0,
                    "csm_status_timer_interval_ms": 50,
                    "registration_retry_ms": 100,
                }
            ],
            output="screen",
        )
        for name, priority in PRIORITIES.items()
    ]
    # launch_testing runs active unittest methods on a background thread.
    # A separate interpreter keeps QApplication and every widget on its real
    # OS main thread; this file remains the only test helper.
    scenario = ExecuteProcess(
        cmd=[
            sys.executable,
            os.path.abspath(__file__),
            LaunchConfiguration("gui_scenario_server_pid"),
        ],
        additional_env={"QT_QPA_PLATFORM": "offscreen"},
        output="screen",
    )
    scenario_started = False

    def start_scenario(event, context):
        nonlocal scenario_started
        if scenario_started:
            return []
        scenario_started = True
        return [
            SetLaunchConfiguration("gui_scenario_server_pid", str(event.pid)),
            scenario,
        ]

    start_handler = RegisterEventHandler(
        OnProcessStart(target_action=server, on_start=start_scenario)
    )
    return LaunchDescription(
        [start_handler, master, server, *bridges, ReadyToTest()]
    ), {
        "master": master,
        "server": server,
        "bridges": bridges,
        "scenario": scenario,
    }


def stamp_ns(stamp):
    """Compare all timestamp bits, including nanoseconds."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def output_metadata(status):
    """Exclude query time so repeated polling cannot conceal an output change."""
    return (
        status.has_output,
        stamp_ns(status.output_stamp),
        status.output_controller_name,
        status.output_api_id,
        status.output_parameter,
        bytes(status.output_binary),
    )


class GuiScenario:
    """Run Qt widgets and the asynchronous ROS client on the OS main thread."""

    @classmethod
    def setUpClass(cls):
        """Create a Qt application, actual service client and independent observers."""
        if threading.current_thread() is not threading.main_thread():
            raise AssertionError(
                "QApplication must be constructed on the OS main thread"
            )
        init()
        cls.app = QApplication.instance() or QApplication(
            ["control_server_gui_integration", "-platform", "offscreen"]
        )
        cls.client = ControlServerGuiNode(
            parameter_overrides=[
                Parameter("server_name", value=SERVER),
                Parameter("poll_interval_ms", value=50),
                Parameter("service_timeout_ms", value=300),
            ]
        )
        cls.window = ControlServerWindow(cls.client)
        cls.window.show()
        cls.probe = create_node("r1_gui_integration_probe")
        cls.enabled = {HIGH: False, LOW: False}
        cls.requests = []
        cls.manager = None
        cls.publishers = {
            name: cls.probe.create_publisher(
                Joy, f"/{SERVER}/{name}/joy", qos_profile_sensor_data
            )
            for name in PRIORITIES
        }
        cls.request_sub = cls.probe.create_subscription(
            Request, "/api/sport/request", cls.requests.append, 100
        )
        cls.manager_sub = cls.probe.create_subscription(
            ManagerStatus, f"/{SERVER}/status", cls.record_manager, 10
        )
        cls.wireless = cls.probe.create_publisher(
            WirelessController, "/wirelesscontroller", qos_profile_sensor_data
        )
        cls.timer = cls.probe.create_timer(0.05, cls.publish_inputs)
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.client)
        cls.executor.add_node(cls.probe)

    @classmethod
    def tearDownClass(cls):
        """Close the production client before destroying its executor and widgets."""
        cls.enabled = {HIGH: False, LOW: False}
        cls.window.close()
        cls.client.close()
        cls.executor.shutdown()
        cls.client.destroy_node()
        cls.probe.destroy_node()
        shutdown()
        cls.app.processEvents()

    @classmethod
    def record_manager(cls, message):
        """Retain actual transport state for readiness and rejection assertions."""
        cls.manager = message

    @classmethod
    def publish_inputs(cls):
        """Model a joystick's autorepeat stream; disabled means genuine silence."""
        for name, enabled in cls.enabled.items():
            if enabled:
                message = Joy()
                message.header.stamp = cls.probe.get_clock().now().to_msg()
                message.axes = [SPEEDS[name], 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
                message.buttons = [0] * 12
                cls.publishers[name].publish(message)

    def pump(self):
        """Advance ROS callbacks, client deadlines and Qt events without a thread."""
        self.executor.spin_once(timeout_sec=0.01)
        self.client.tick()
        self.window.render()
        self.app.processEvents()

    def wait_for(self, predicate, timeout=15.0):
        """Wait for an observed condition while keeping the whole GUI responsive."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            if predicate():
                return True
        return predicate()

    def pump_for(self, duration):
        """Continue processing timers and service replies during a quiet interval."""
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.pump()

    def sink_state(self, name, state):
        """Require the server's transport endpoint to reach the requested state."""
        return bool(
            self.manager
            and any(
                entry.controller_name == name and entry.state == state
                for entry in self.manager.sinks
            )
        )

    def select(self, name):
        """Select the visible table cell with an actual Qt mouse event."""
        table = self.window.controller_table
        for row in range(table.rowCount()):
            item = table.item(row, 0)
            if item.text() == name:
                table.scrollToItem(item)
                QTest.mouseClick(
                    table.viewport(),
                    Qt.LeftButton,
                    pos=table.visualItemRect(item).center(),
                )
                self.assertEqual(self.window.selected_controller(), name)
                return
        self.fail(f"{name} is absent from the registered-controller table")

    def click(self, button, success):
        """Send a real widget operation and require its asynchronous typed result."""
        self.assertTrue(button.isEnabled())
        QTest.mouseClick(button, Qt.LeftButton)
        self.assertTrue(self.client.state.operation_pending)
        self.assertTrue(
            self.wait_for(lambda: not self.client.state.operation_pending),
            "GUI operation never completed",
        )
        self.assertIs(self.client.state.operation_success, success)
        self.assertEqual(
            self.window.result_label.text(), self.client.state.operation_message
        )

    def assert_output(self, count, api, controller):
        """Compare status and rendered output against the actual published request."""
        self.assertTrue(
            self.wait_for(
                lambda: (
                    len(self.requests) >= count
                    and self.client.state.connected
                    and self.client.state.status.output_api_id == api
                    and self.client.state.status.output_controller_name == controller
                    and self.client.state.status.output_parameter
                    == self.requests[-1].parameter
                )
            ),
            f"GUI did not observe output {count}: API={api}, controller={controller}",
        )
        self.assertEqual(len(self.requests), count)
        request, status = self.requests[-1], self.client.state.status
        self.assertEqual(request.header.identity.api_id, api)
        self.assertTrue(status.has_output)
        self.assertGreater(stamp_ns(status.output_stamp), 0)
        self.assertEqual(status.output_api_id, request.header.identity.api_id)
        self.assertEqual(status.output_parameter, request.parameter)
        self.assertEqual(bytes(status.output_binary), bytes(request.binary))
        self.assertEqual(
            self.window.output_controller_label.text(),
            controller or "Server / no controller",
        )
        self.assertIn(str(api), self.window.api_label.text())
        if request.parameter:
            self.assertEqual(
                json.loads(self.window.payload_text.toPlainText()),
                json.loads(request.parameter),
            )
        else:
            self.assertEqual(self.window.payload_text.toPlainText(), "")
        self.assertEqual(
            self.window.binary_text.toPlainText(), bytes(request.binary).hex(" ")
        )

    def assert_quiet(self, duration=0.25):
        """Poll and render without publishing or changing last-output metadata."""
        before = output_metadata(self.client.state.status)
        stamp = stamp_ns(self.client.state.status.stamp)
        count = len(self.requests)
        self.pump_for(duration)
        self.assertEqual(len(self.requests), count)
        self.assertEqual(output_metadata(self.client.state.status), before)
        self.assertGreater(stamp_ns(self.client.state.status.stamp), stamp)

    def test_live_services_switch_interrupt_and_reconnect(self):
        """Drive the complete GUI lifecycle without mocks or robot communication."""
        self.assertTrue(
            self.wait_for(
                lambda: (
                    self.client.state.connected
                    and all(
                        pub.get_subscription_count() for pub in self.publishers.values()
                    )
                    and self.request_sub.get_publisher_count() > 0
                    and self.wireless.get_subscription_count() > 0
                )
            )
        )
        self.assertEqual(self.client.target_prefix, f"/{SERVER}")
        self.assertEqual(self.window.connection_label.text(), "Connected")
        self.assertFalse(self.client.state.status.has_output)
        self.assertEqual(
            self.window.output_controller_label.text(), "No output published"
        )
        self.assertEqual(self.window.controller_table.rowCount(), 0)
        self.assertFalse(self.window.switch_button.isEnabled())
        self.assertFalse(self.window.reset_button.isEnabled())
        self.assert_quiet()

        self.enabled[HIGH] = True
        self.assert_output(1, MOVE, HIGH)
        self.assertAlmostEqual(
            json.loads(self.requests[-1].parameter)["x"], SPEEDS[HIGH]
        )
        self.enabled[LOW] = True
        self.assertTrue(
            self.wait_for(
                lambda: (
                    self.window.controller_table.rowCount() == 2
                    and self.sink_state(LOW, EntryStatus.STATE_ACTIVE)
                    and self.window.active_label.text() == HIGH
                )
            )
        )
        table = self.window.controller_table
        for row in range(table.rowCount()):
            name = table.item(row, 0).text()
            self.assertEqual(table.item(row, 1).text(), "joy")
            self.assertEqual(table.item(row, 2).text(), str(PRIORITIES[name]))
            self.assertEqual(table.item(row, 3).text(), f"/{SERVER}/{name}/control")
            self.assertEqual(
                table.item(row, 4).text(), "Current" if name == HIGH else ""
            )
        self.assert_quiet()

        # Let queued low input drain while the endpoint remains locally fresh.
        # A successful manual switch emits only the old owner's stop until a
        # genuinely new low-priority joystick packet arrives.
        self.enabled[LOW] = False
        self.pump_for(0.15)
        self.select(LOW)
        self.click(self.window.switch_button, True)
        self.assertTrue(self.wait_for(lambda: self.window.active_label.text() == LOW))
        self.assert_output(2, STOP, HIGH)
        self.assert_quiet(0.15)
        self.enabled[LOW] = True
        self.assert_output(3, MOVE, LOW)
        self.assertAlmostEqual(
            json.loads(self.requests[-1].parameter)["x"], SPEEDS[LOW]
        )

        # This row stays registered after data timeout. The actual server must
        # reject its switch request, and the GUI must show that typed failure.
        self.enabled[LOW] = False
        self.assertTrue(
            self.wait_for(
                lambda: (
                    self.sink_state(LOW, EntryStatus.STATE_TIMEOUT)
                    and self.window.active_label.text() == HIGH
                    and len(self.requests) >= 5
                )
            )
        )
        self.assertEqual(self.requests[3].header.identity.api_id, STOP)
        self.assert_output(5, MOVE, HIGH)
        self.select(LOW)
        self.click(self.window.switch_button, False)
        self.assertIn("not ACTIVE", self.window.result_label.text())
        self.assertEqual(self.window.active_label.text(), HIGH)
        self.assert_quiet()

        self.wireless.publish(WirelessController())
        self.assertTrue(self.wait_for(lambda: self.client.state.status.interrupted))
        self.assert_output(6, STOP, "")
        self.assertIn("latched", self.window.interrupt_label.text())
        self.assertFalse(self.window.switch_button.isEnabled())
        self.assertTrue(self.window.reset_button.isEnabled())
        for _ in range(3):
            self.wireless.publish(WirelessController())
            self.pump_for(0.05)
        self.assert_quiet()

        self.enabled[HIGH] = False
        self.pump_for(0.15)
        self.click(self.window.reset_button, True)
        self.assertTrue(
            self.wait_for(
                lambda: (
                    not self.client.state.status.interrupted
                    and not self.client.state.status.active_controller_name
                )
            )
        )
        self.assertIn("clear", self.window.interrupt_label.text())
        self.assertFalse(self.window.reset_button.isEnabled())
        self.assert_quiet()
        self.enabled[LOW] = True
        self.assert_output(7, MOVE, LOW)

        # The client's timeout must disable controls before launch respawns
        # the real server. Old snapshots remain visibly stale, never actionable.
        self.select(LOW)
        self.assertTrue(self.window.switch_button.isEnabled())
        before = len(self.requests)
        os.kill(int(sys.argv[1]), signal.SIGKILL)
        self.assertTrue(
            self.wait_for(lambda: not self.client.state.connected, timeout=1.5),
            "service loss never disabled the GUI before server restart",
        )
        self.assertFalse(self.window.switch_button.isEnabled())
        self.assertFalse(self.window.reset_button.isEnabled())
        self.assertIn("STALE", self.window.snapshot_label.text())
        self.assertIn("controls disabled", self.window.interrupt_label.text())
        self.assertTrue(
            self.wait_for(
                lambda: (
                    self.client.state.connected
                    and self.client.state.status.active_controller_name == LOW
                    and len(self.requests) > before
                ),
                timeout=25.0,
            ),
            "GUI did not recover with the restarted server and live bridge",
        )
        self.assert_output(before + 1, MOVE, LOW)
        self.select(LOW)
        self.assertTrue(self.window.switch_button.isEnabled())
        self.assertEqual(self.window.connection_label.text(), "Connected")
        self.assertNotIn("STALE", self.window.snapshot_label.text())
        self.assert_quiet()


class TestGuiScenarioProcess(unittest.TestCase):
    """Monitor the independent main-thread Qt test without touching its widgets."""

    def test_scenario_completed(self, proc_info, scenario):
        """Require every real-service and widget assertion in the child to pass."""
        proc_info.assertWaitForStartup(process=scenario, timeout=20.0)
        proc_info.assertWaitForShutdown(process=scenario, timeout=140.0)
        self.assertEqual(proc_info[scenario].returncode, 0)


@post_shutdown_test()
class TestGuiPeerShutdown(unittest.TestCase):
    """The final restarted server and both transport peers shut down cleanly."""

    def test_exit_codes(self, proc_info):
        """Ignore no crashes other than the intentional earlier respawn trigger."""
        assertExitCodes(proc_info)


if __name__ == "__main__":
    scenario_type = type("TestControlServerGui", (GuiScenario, unittest.TestCase), {})
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(scenario_type)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
