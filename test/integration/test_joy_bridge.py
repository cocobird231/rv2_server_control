"""Exercise real R1 bridge/server processes, Unitree output, loss and recovery."""

import json
import os
import signal
import threading
import time
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing import post_shutdown_test
from launch_testing.actions import ReadyToTest
from launch_testing.asserts import assertExitCodes
from r1_interfaces.msg import ManagerStatus
from rclpy import create_node, init, shutdown
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from unitree_api.msg import Request

IDLE_AXES = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
MOVE, STOP = 1008, 1003


def generate_test_description():
    """Start all peers immediately; the test waits for observed data readiness."""
    master = Node(
        package="rv2_control_signal_transport",
        executable="csm_master_node",
        parameters=[{"tick_interval_ms": 50, "pair_grace_ms": 150}],
        output="screen",
    )
    server = Node(
        package="rv2_server_control",
        executable="control_server",
        name="control_server",
        parameters=[{"server_name": "r1_server_test", "status_timer_interval_ms": 50}],
        output="screen",
        respawn=True,
        respawn_delay=0.2,
    )
    bridge = Node(
        package="rv2_csm_topic_bridge",
        executable="topic_bridge",
        name="topic_bridge",
        parameters=[
            {
                "topic_name": "/r1_server_test/joy",
                "server_name": "r1_server_test",
                "csm_name": "r1_bridge_test",
                "controller_name": "r1_joy_test",
                "channel_name": "/r1_server_test/control",
                "priority": 100,
                "timeout_ms": 300,
                "disconnect_timeout_ms": 1200,
                "csm_status_timer_interval_ms": 50,
            }
        ],
        output="screen",
        respawn=True,
        respawn_delay=0.2,
    )
    return LaunchDescription([bridge, master, server, ReadyToTest()]), {
        "bridge": bridge,
        "server": server,
        "master": master,
    }


class TestJoyPipeline(unittest.TestCase):
    """Drive a joystick stream without requiring a physical device or robot."""

    @classmethod
    def setUpClass(cls):
        """Create an independent best-effort joy publisher and output recorder."""
        init()
        cls.node = create_node("r1_joy_pipeline_probe")
        cls.lock = threading.Lock()
        cls.axes = list(IDLE_AXES)
        cls.axes[0] = 0.2
        cls.buttons = [0] * 12
        cls.enabled = False
        cls.requests = []
        cls.statuses = {}
        cls.publisher = cls.node.create_publisher(
            Joy, "/r1_server_test/joy", qos_profile_sensor_data
        )
        cls.request_sub = cls.node.create_subscription(
            Request, "/api/sport/request", cls.record_request, 100
        )
        cls.status_subs = [
            cls.node.create_subscription(ManagerStatus, topic, cls.record_status, 10)
            for topic in ("/r1_server_test/status", "/r1_bridge_test/status")
        ]
        cls.timer = cls.node.create_timer(0.05, cls.publish_joy)
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        """Drain the test executor before destroying entities."""
        cls.executor.shutdown()
        cls.thread.join()
        cls.node.destroy_node()
        shutdown()

    @classmethod
    def publish_joy(cls):
        """Publish only new joystick input; disabled means real input silence."""
        with cls.lock:
            if not cls.enabled:
                return
            message = Joy()
            message.axes = list(cls.axes)
            message.buttons = list(cls.buttons)
        message.header.stamp = cls.node.get_clock().now().to_msg()
        cls.publisher.publish(message)

    @classmethod
    def record_request(cls, message):
        """Record observable Unitree requests with local receipt time."""
        with cls.lock:
            cls.requests.append((time.monotonic(), message))

    @classmethod
    def record_status(cls, message):
        """Track each actual R1 manager incarnation and its endpoint snapshot."""
        with cls.lock:
            cls.statuses[message.manager_name] = message

    def wait_for(self, predicate, timeout=20.0):
        """Bound readiness by observable evidence rather than a startup sleep."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.02)
        return predicate()

    def count(self, api):
        """Read the number of a specific sport API request."""
        with self.lock:
            return sum(
                message.header.identity.api_id == api for _, message in self.requests
            )

    def latest_move(self):
        """Return the most recent movement payload."""
        with self.lock:
            for _, message in reversed(self.requests):
                if message.header.identity.api_id == MOVE:
                    return json.loads(message.parameter)
        return {}

    def update(self, axes=None, button=None, enabled=None):
        """Atomically change the simulated physical joystick state."""
        with self.lock:
            if axes is not None:
                self.__class__.axes = list(axes)
            if button is not None:
                self.__class__.buttons = [int(index == button) for index in range(12)]
            if enabled is not None:
                self.__class__.enabled = enabled

    def incarnation(self, name):
        """Read a manager's incarnation from the actual wire status."""
        with self.lock:
            status = self.statuses.get(name)
            return status.csm_instance_id if status else None

    def test_data_stop_reconnect_and_peer_restart(self, proc_info, bridge, server):
        """Verify translation, no stale replay, loss edge, and both peer restarts."""
        # The first Move is emitted only once for held axes. Match both ends
        # before publishing so the observer cannot miss that initial event.
        self.assertTrue(
            self.wait_for(
                lambda: (
                    self.publisher.get_subscription_count() > 0
                    and self.request_sub.get_publisher_count() > 0
                )
            ),
            "Joy input or Unitree request observer did not match",
        )
        self.update(enabled=True)
        self.assertTrue(
            self.wait_for(lambda: self.count(MOVE) > 0), "no initial R1 output"
        )
        self.assertTrue(
            self.wait_for(lambda: self.incarnation("r1_bridge_test") is not None)
        )
        before = self.count(MOVE)
        axes = list(IDLE_AXES)
        axes[0], axes[1], axes[3] = 0.5, -0.2, 0.3
        self.update(axes=axes)
        self.assertTrue(self.wait_for(lambda: self.count(MOVE) > before))
        payload = self.latest_move()
        self.assertAlmostEqual(payload["x"], 0.5, places=5)
        self.assertAlmostEqual(payload["y"], -0.2, places=5)
        self.assertAlmostEqual(payload["z"], 0.3, places=5)
        before = self.count(MOVE)
        time.sleep(0.35)
        self.assertEqual(self.count(MOVE), before, "held axes bypassed dedup")

        self.update(axes=IDLE_AXES)
        self.assertTrue(self.wait_for(lambda: self.count(MOVE) > before))
        for button, api in enumerate((1004, 1005, 1003, 1006)):
            before = self.count(api)
            self.update(button=button)
            self.assertTrue(self.wait_for(lambda: self.count(api) > before))
            time.sleep(0.15)
            self.assertEqual(self.count(api), before + 1, "button repeated while held")
            self.update(button=-1)
            time.sleep(0.1)

        # Disconnect the input while bridge remains alive. It must not keep
        # forwarding a cached sample and thereby conceal the missing joystick.
        self.update(axes=axes)
        self.assertTrue(
            self.wait_for(lambda: abs(self.latest_move().get("x", 0) - 0.5) < 0.001)
        )
        before_stop = self.count(STOP)
        self.update(enabled=False)
        self.assertTrue(self.wait_for(lambda: self.count(STOP) == before_stop + 1, 5.0))
        before_move = self.count(MOVE)
        time.sleep(1.5)
        self.assertEqual(self.count(STOP), before_stop + 1, "loss edge repeated")
        self.assertEqual(self.count(MOVE), before_move, "stale input was replayed")
        self.update(enabled=True)
        self.assertTrue(
            self.wait_for(lambda: self.count(MOVE) > before_move),
            "same-value reconnect stayed deduped",
        )

        # Kill a real bridge process; launch respawns a new manager incarnation.
        before_instance = self.incarnation("r1_bridge_test")
        os.kill(proc_info[bridge].pid, signal.SIGKILL)
        self.assertTrue(
            self.wait_for(
                lambda: (
                    self.incarnation("r1_bridge_test") not in (None, before_instance)
                )
            )
        )
        before_move = self.count(MOVE)
        axes[0] = 0.61
        self.update(axes=axes)
        self.assertTrue(
            self.wait_for(
                lambda: (
                    self.count(MOVE) > before_move
                    and abs(self.latest_move().get("x", 0) - 0.61) < 0.001
                )
            ),
            "bridge restart did not deliver the new marker",
        )

        # Target restart requires the real master to reconcile the still-live
        # source. Compare incarnation and actual output, never only process logs.
        before_instance = self.incarnation("r1_server_test")
        os.kill(proc_info[server].pid, signal.SIGKILL)
        self.assertTrue(
            self.wait_for(
                lambda: (
                    self.incarnation("r1_server_test") not in (None, before_instance)
                )
            )
        )
        before_move = self.count(MOVE)
        axes[0] = 0.72
        self.update(axes=axes)
        self.assertTrue(
            self.wait_for(
                lambda: (
                    self.count(MOVE) > before_move
                    and abs(self.latest_move().get("x", 0) - 0.72) < 0.001
                )
            ),
            "server restart did not deliver the new marker",
        )


@post_shutdown_test()
class TestProcessExit(unittest.TestCase):
    """Check the final managed processes exit, including intentional crash tests."""

    def test_exit_codes(self, proc_info, master, server, bridge):
        """Require clean final exits; proc_info tracks each respawned process."""
        assertExitCodes(proc_info, process=master)
        for process in (server, bridge):
            assertExitCodes(proc_info, process=process, allowable_exit_codes=[0])
