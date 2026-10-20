"""Exercise production control services, global ownership and wireless latching."""

import json
import threading
import time
import unittest
import uuid

from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing import post_shutdown_test
from launch_testing.actions import ReadyToTest
from launch_testing.asserts import assertExitCodes
from r1_interfaces.msg import ControlSignalInfo, EntryStatus, ManagerStatus
from r1_interfaces.srv import (
    ControlServerStatusReq,
    ControlSignalJoy,
    ControlSignalManage,
    ControlSignalTwist,
    CsmRegister,
    ResetInterruptReq,
    SwitchControllerReq,
)
from rclpy import create_node, init, shutdown
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from unitree_api.msg import Request
from unitree_go.msg import WirelessController

SERVER, MASTER, SOURCE = (
    "r1_services_server",
    "r1_services_master",
    "r1_services_source",
)
JOY, TWIST, IDLE, STRING, LATE = "joy_owner", "twist_owner", "idle", "string", "late"
MOVE, STOP = 1008, 1003
DATA_TIMEOUT = 2.0


def generate_test_description():
    """Launch actual nodes; the Python probe implements only the source protocol."""
    master = Node(
        package="rv2_control_signal_transport",
        executable="csm_master_node",
        parameters=[{"master_name": MASTER, "tick_interval_ms": 50}],
        output="screen",
    )
    server = Node(
        package="rv2_server_control",
        executable="control_server",
        name="control_services_server",
        parameters=[
            {
                "server_name": SERVER,
                "master_name": MASTER,
                "status_timer_interval_ms": 50,
                "watchdog_interval_ms": 20,
            }
        ],
        output="screen",
    )
    return LaunchDescription([master, server, ReadyToTest()]), {
        "master": master,
        "server": server,
    }


def stamp_ns(stamp):
    """Compare the full ROS timestamp rather than rounding its seconds."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class TestControlServices(unittest.TestCase):
    """Use acknowledged data services to distinguish fresh input from cached data."""

    @classmethod
    def setUpClass(cls):
        """Create observers and a source peer with real registration snapshots."""
        init()
        cls.node = create_node("r1_control_services_probe")
        cls.lock = threading.Lock()
        cls.requests = []
        cls.entries = {}
        cls.sent_at = {}
        cls.data_clients = {}
        cls.manager_status = None
        cls.instance = str(uuid.uuid4())
        cls.sequence = 0
        cls.source_ready = False
        cls.source_status = cls.node.create_publisher(
            ManagerStatus, f"/{SOURCE}/status", 10
        )
        cls.request_sub = cls.node.create_subscription(
            Request, "/api/sport/request", cls.record_request, 100
        )
        cls.manager_sub = cls.node.create_subscription(
            ManagerStatus, f"/{SERVER}/status", cls.record_manager, 10
        )
        cls.wireless = cls.node.create_publisher(
            WirelessController, "/wirelesscontroller", qos_profile_sensor_data
        )
        cls.status_client = cls.node.create_client(
            ControlServerStatusReq, f"/{SERVER}/control_server_status"
        )
        cls.switch_client = cls.node.create_client(
            SwitchControllerReq, f"/{SERVER}/switch_controller"
        )
        cls.reset_client = cls.node.create_client(
            ResetInterruptReq, f"/{SERVER}/reset_interrupt"
        )
        cls.register_client = cls.node.create_client(
            ControlSignalManage, f"/{SERVER}/control_signal_manage"
        )
        cls.master_client = cls.node.create_client(CsmRegister, f"/{MASTER}/register")
        cls.timer = cls.node.create_timer(0.05, cls.publish_sources)
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        """Stop the probe before destroying its subscriptions and service clients."""
        cls.executor.shutdown()
        cls.thread.join()
        cls.node.destroy_node()
        shutdown()

    @classmethod
    def record_request(cls, message):
        """Capture the actual published Unitree payload independently of status."""
        with cls.lock:
            cls.requests.append(message)

    @classmethod
    def record_manager(cls, message):
        """Keep endpoint ACTIVE evidence for the manual-switch service gate."""
        with cls.lock:
            cls.manager_status = message

    @classmethod
    def publish_sources(cls):
        """Keep real source/sink identities paired while input itself may go silent."""
        with cls.lock:
            if not cls.source_ready:
                return
            cls.sequence += 1
            message = ManagerStatus()
            message.manager_name = SOURCE
            message.csm_instance_id = cls.instance
            message.snapshot_seq = cls.sequence
            message.snapshot_ready = True
            message.stamp = cls.node.get_clock().now().to_msg()
            for name, entry in cls.entries.items():
                sent = cls.sent_at.get(name)
                entry.state = (
                    EntryStatus.STATE_INITIAL
                    if sent is None
                    else EntryStatus.STATE_ACTIVE
                    if time.monotonic() - sent < DATA_TIMEOUT
                    else EntryStatus.STATE_TIMEOUT
                )
                message.sources.append(entry)
            cls.source_status.publish(message)

    def wait_for(self, predicate, timeout=10.0):
        """Bound discovery and progress by observed state, not fixed startup delays."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.01)
        return predicate()

    def call(self, client, request):
        """Wait on the independent executor and preserve service error diagnostics."""
        self.assertTrue(client.wait_for_service(timeout_sec=10.0), client.srv_name)
        future = client.call_async(request)
        self.assertTrue(self.wait_for(future.done), client.srv_name)
        self.assertIsNone(future.exception())
        return future.result()

    def status(self):
        """Read the production status RPC, requiring its typed success result."""
        response = self.call(self.status_client, ControlServerStatusReq.Request())
        self.assertEqual(response.response, response.RESPONSE_SUCCESS, response.reason)
        self.assertEqual(response.status.server_name, SERVER)
        self.assertGreater(stamp_ns(response.status.stamp), 0)
        return response.status

    def register_controller(self, name, kind, priority):
        """Register a real sink, publishing its pending and committed source identity."""
        request = ControlSignalManage.Request()
        request.op = request.OP_REGISTER
        request.source_manager_name = SOURCE
        request.source_csm_instance_id = self.instance
        request.registration_id = str(uuid.uuid4())
        request.attempt_generation = 1
        info = ControlSignalInfo()
        info.controller_name = name
        info.channel_name = f"/{SERVER}/{name}/data"
        info.target_manager_name = SERVER
        info.type = kind
        info.mode = "topic" if kind == "string" else "service"
        info.priority = priority
        info.timeout_ns = int(DATA_TIMEOUT * 1_000_000_000)
        info.disconnect_timeout_ns = 0
        request.info = info
        entry = EntryStatus()
        entry.manager_name = entry.source_manager_name = SOURCE
        entry.csm_instance_id = entry.source_csm_instance_id = self.instance
        entry.target_manager_name = SERVER
        entry.registration_id = request.registration_id
        entry.attempt_generation = 1
        entry.controller_name, entry.channel_name = name, info.channel_name
        entry.type, entry.mode, entry.priority = kind, info.mode, priority
        entry.is_source = True
        entry.registration_phase = entry.PHASE_PENDING
        with self.lock:
            self.entries[name] = entry
        response = self.call(self.register_client, request)
        self.assertEqual(response.response, response.RESPONSE_SUCCESS, response.reason)
        with self.lock:
            entry.registration_phase = entry.PHASE_REGISTERED
            entry.endpoint_present = True
        if kind != "string":
            service = ControlSignalJoy if kind == "joy" else ControlSignalTwist
            client = self.node.create_client(service, info.channel_name)
            self.assertTrue(client.wait_for_service(timeout_sec=10.0))
            self.data_clients[name] = (service, client)

    def send(self, name, message):
        """Fence the production receive callback with a data-service reply."""
        service, client = self.data_clients[name]
        request = service.Request()
        request.data = message
        with self.lock:
            self.sent_at[name] = time.monotonic()
        response = self.call(client, request)
        self.assertEqual(response.response, response.SRV_RES_SUCCESS, response.reason)

    def joy(self, value=0.55, *, request=False, emergency=False):
        """Generate valid released triggers, with optional arbitration sentinels."""
        message = Joy()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.axes = [value, -0.2, 1.0, 0.3, 0.0, -1.0 if emergency else 1.0]
        message.buttons = [0] * 12
        message.buttons[11] = int(request)
        return message

    def switch(self, name, expected):
        """Address controller identity, not channel, and verify the typed result."""
        request = SwitchControllerReq.Request()
        request.controller_name = name
        response = self.call(self.switch_client, request)
        self.assertEqual(response.response, expected, response.reason)
        self.assertEqual(bool(response.reason), expected != response.RESPONSE_SUCCESS)

    def reset(self):
        """Reset is an idempotent explicit command with no request arguments."""
        response = self.call(self.reset_client, ResetInterruptReq.Request())
        self.assertEqual(response.response, response.RESPONSE_SUCCESS, response.reason)

    def count(self):
        """Read the observer count without racing its executor callback."""
        with self.lock:
            return len(self.requests)

    def assert_quiet(self, count, duration=0.15):
        """Observe a full no-publication window, including multiple watchdog ticks."""
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.assertEqual(self.count(), count, "unexpected Unitree output")
            time.sleep(0.01)
        self.assertEqual(self.count(), count)

    def assert_held_input(self, name, message_factory, count):
        """Keep input alive across real status ticks without producing more output."""
        before = self.status()
        with self.lock:
            sequence = self.manager_status.snapshot_seq
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            self.send(name, message_factory())
            self.assert_quiet(count, 0.025)

        def ticks_advanced():
            with self.lock:
                return self.manager_status.snapshot_seq >= sequence + 3

        self.assertTrue(self.wait_for(ticks_advanced), "Manager status stopped ticking")
        self.assertTrue(
            self.sink_active(name), "held input did not keep its sink alive"
        )
        after = self.status()
        self.assertEqual(after.active_controller_name, name)
        self.assertGreater(stamp_ns(after.stamp), stamp_ns(before.stamp))
        self.assertEqual(self.output_metadata(after), self.output_metadata(before))
        self.assertEqual(self.count(), count)

    @staticmethod
    def output_metadata(status):
        """Exclude query time so suppressed input must preserve all output fields."""
        return (
            status.has_output,
            stamp_ns(status.output_stamp),
            status.output_controller_name,
            status.output_api_id,
            status.output_parameter,
            bytes(status.output_binary),
        )

    def assert_output(self, count, api, controller):
        """Match last-output metadata to the independent ROS topic observer."""
        self.assertTrue(self.wait_for(lambda: self.count() >= count), "missing output")
        self.assertEqual(self.count(), count)
        with self.lock:
            message = self.requests[-1]
        status = self.status()
        self.assertTrue(status.has_output)
        self.assertGreater(stamp_ns(status.output_stamp), 0)
        self.assertLessEqual(stamp_ns(status.output_stamp), stamp_ns(status.stamp))
        self.assertEqual(status.output_controller_name, controller)
        self.assertEqual(status.output_api_id, api)
        self.assertEqual(message.header.identity.api_id, api)
        self.assertEqual(status.output_parameter, message.parameter)
        self.assertEqual(bytes(status.output_binary), bytes(message.binary))
        return status

    def sink_active(self, name):
        """Require the Manager's actual ACTIVE tick before manual selection."""
        with self.lock:
            return self.manager_status is not None and any(
                entry.controller_name == name and entry.state == entry.STATE_ACTIVE
                for entry in self.manager_status.sinks
            )

    def interrupt(self):
        """Send the same all-zero packet; every received packet must latch."""
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            self.wireless.publish(WirelessController())
            if self.status().interrupted:
                return
            time.sleep(0.02)
        self.fail("wireless packet did not latch the production server")

    def test_services_global_owner_and_latched_wireless(self):
        """Check real registration, handoff, output metadata and explicit recovery."""
        result = SwitchControllerReq.Response
        self.assertTrue(
            self.wait_for(lambda: self.request_sub.get_publisher_count() > 0)
        )
        self.assertTrue(
            self.wait_for(lambda: self.wireless.get_subscription_count() > 0)
        )
        initial = self.status()
        self.assertFalse(initial.interrupted)
        self.assertEqual(initial.active_controller_name, "")
        self.assertEqual(initial.registered_controllers, [])
        self.assertEqual(self.output_metadata(initial), (False, 0, "", 0, "", b""))
        self.assertEqual(self.count(), 0)

        peer = CsmRegister.Request()
        peer.csm_name, peer.csm_instance_id = SOURCE, self.instance
        peer.status_interval_ns = 50_000_000
        peer.registration_grace_ns = 1_000_000_000
        # This controlled peer has no heartbeat timeout; snapshots still drive
        # real master pairing. Data timeout is independently enabled on each sink.
        registered = self.call(self.master_client, peer)
        self.assertEqual(
            registered.response, registered.RESPONSE_SUCCESS, registered.reason
        )
        with self.lock:
            self.__class__.source_ready = True
        for name, kind, priority in (
            (JOY, "joy", 80),
            (TWIST, "twist", 20),
            (IDLE, "joy", 40),
            (STRING, "string", 10),
        ):
            self.register_controller(name, kind, priority)
        registered = self.status()
        self.assertEqual(
            {info.controller_name for info in registered.registered_controllers},
            {JOY, TWIST, IDLE, STRING},
        )
        descriptors = {
            info.controller_name: info for info in registered.registered_controllers
        }
        self.assertEqual(descriptors[JOY].priority, 80)
        self.assertEqual(descriptors[TWIST].priority, 20)
        self.assertEqual(descriptors[TWIST].type, "twist")
        self.assertEqual(descriptors[JOY].channel_name, f"/{SERVER}/{JOY}/data")
        self.assertEqual(descriptors[JOY].timeout_ns, int(DATA_TIMEOUT * 1_000_000_000))
        self.assertFalse(registered.has_output)
        self.assertEqual(registered.active_controller_name, "")
        self.switch("missing", result.RESPONSE_NOT_FOUND)
        self.switch(IDLE, result.RESPONSE_NOT_ACTIVE)
        self.switch(STRING, result.RESPONSE_UNSUPPORTED_TYPE)

        self.send(JOY, self.joy())
        first = self.assert_output(1, MOVE, JOY)
        self.assertEqual(first.active_controller_name, JOY)
        payload = json.loads(first.output_parameter)
        for axis, value in (("x", 0.55), ("y", -0.2), ("z", 0.3)):
            self.assertAlmostEqual(payload[axis], value, places=5)
        self.send(JOY, self.joy())
        self.assert_quiet(1)
        self.assertEqual(
            self.output_metadata(self.status()), self.output_metadata(first)
        )

        twist = Twist()
        twist.linear.x, twist.angular.z = 0.2, 0.1
        self.send(TWIST, twist)
        self.assertTrue(self.wait_for(lambda: self.sink_active(TWIST)))
        self.assertEqual(self.status().active_controller_name, JOY)
        self.assert_quiet(1)
        self.switch(TWIST, result.RESPONSE_SUCCESS)
        stopped = self.assert_output(2, STOP, JOY)
        self.assertEqual(stopped.active_controller_name, TWIST)
        self.assert_quiet(2)  # Switching alone cannot replay the cached Twist.
        self.send(JOY, self.joy(0.7))  # Higher ordinary Joy cannot steal ownership.
        self.assert_quiet(2)
        self.send(TWIST, twist)
        moved = self.assert_output(3, MOVE, TWIST)
        self.assertEqual(moved.active_controller_name, TWIST)
        self.send(JOY, self.joy(request=True))
        self.assert_output(4, STOP, TWIST)
        self.assertEqual(self.status().active_controller_name, JOY)
        self.send(JOY, self.joy())
        self.assert_output(5, MOVE, JOY)

        self.interrupt()
        interrupted = self.assert_output(6, STOP, "")
        self.assertTrue(interrupted.interrupted)
        self.assertEqual(interrupted.active_controller_name, "")
        for _ in range(3):
            self.wireless.publish(WirelessController())
        self.send(JOY, self.joy(0.8))
        self.send(JOY, self.joy(request=True))
        self.send(JOY, self.joy(emergency=True))
        self.send(TWIST, twist)
        for sentinel in (99.0, -99.0):
            control = Twist()
            control.linear.z = control.angular.x = control.angular.y = sentinel
            self.send(TWIST, control)
        self.register_controller(LATE, "joy", 90)
        self.send(LATE, self.joy(0.9))
        self.switch(LATE, result.RESPONSE_INTERRUPTED)
        self.assert_quiet(6, DATA_TIMEOUT + 0.3)
        held = self.status()
        self.assertTrue(held.interrupted)
        self.assertEqual(held.active_controller_name, "")
        self.assertEqual(self.output_metadata(held), self.output_metadata(interrupted))
        self.assertEqual(len(held.registered_controllers), 5)

        # Fresh packets received while latched must not become watchdog fallback
        # after reset, even though their transport endpoints are still ACTIVE.
        self.send(JOY, self.joy())
        self.send(TWIST, twist)
        self.reset()
        self.reset()
        self.assert_quiet(6)
        reset = self.status()
        self.assertFalse(reset.interrupted)
        self.assertEqual(reset.active_controller_name, "")
        self.assertEqual(self.output_metadata(reset), self.output_metadata(interrupted))
        self.switch(JOY, result.RESPONSE_NOT_ACTIVE)
        self.send(JOY, self.joy())  # Same values as the last actual Joy Move.
        resumed = self.assert_output(7, MOVE, JOY)
        self.assertEqual(resumed.active_controller_name, JOY)
        self.assertGreater(
            stamp_ns(resumed.output_stamp), stamp_ns(interrupted.output_stamp)
        )
        self.interrupt()  # An identical neutral wireless packet must re-latch.
        self.assert_output(8, STOP, "")
        self.assert_quiet(8)

        # Continue the same lifecycle so this phase has explicit reset state,
        # rather than relying on unittest's ordering with shared class fixtures.
        self.reset()
        self.assert_quiet(8, 0.3)
        self.assertEqual(self.status().active_controller_name, "")
        self.send(JOY, self.joy(0.61))
        self.assert_output(9, MOVE, JOY)
        self.assert_held_input(JOY, lambda: self.joy(0.61), 9)
        self.send(JOY, self.joy(0.62))
        changed_joy = self.assert_output(10, MOVE, JOY)
        self.assertAlmostEqual(
            json.loads(changed_joy.output_parameter)["x"], 0.62, places=5
        )
        self.assert_held_input(JOY, lambda: self.joy(0.62), 10)

        # A lower-priority Twist first remains pending, then takes over only
        # through the explicit switch. Repeated acknowledged samples test live
        # input deduplication, not the absence of any input callbacks.
        held_twist = Twist()
        held_twist.linear.x, held_twist.angular.z = 0.21, 0.1
        self.send(TWIST, held_twist)
        self.assertTrue(self.wait_for(lambda: self.sink_active(TWIST)))
        self.assertEqual(self.count(), 10)
        self.switch(TWIST, result.RESPONSE_SUCCESS)
        self.assert_output(11, STOP, JOY)
        self.assert_quiet(11)
        self.send(TWIST, held_twist)
        self.assert_output(12, MOVE, TWIST)
        self.assert_held_input(TWIST, lambda: held_twist, 12)
        held_twist.linear.x = 0.22
        self.send(TWIST, held_twist)
        changed_twist = self.assert_output(13, MOVE, TWIST)
        self.assertAlmostEqual(
            json.loads(changed_twist.output_parameter)["x"], 0.22, places=5
        )
        self.assert_held_input(TWIST, lambda: held_twist, 13)

        # Stop all data calls while source snapshots and server timers continue.
        # The active data deadline produces one safety stop, then stays quiet.
        self.assertTrue(
            self.wait_for(lambda: self.count() >= 14, DATA_TIMEOUT + 2.0),
            "silent input did not trigger the safety stop",
        )
        timed_out = self.assert_output(14, STOP, TWIST)
        self.assertEqual(timed_out.active_controller_name, "")
        self.assert_quiet(14, 0.3)
        self.assertEqual(
            self.output_metadata(self.status()), self.output_metadata(timed_out)
        )
        self.send(TWIST, held_twist)
        self.assert_output(15, MOVE, TWIST)
        self.assert_held_input(TWIST, lambda: held_twist, 15)


@post_shutdown_test()
class TestProcessExit(unittest.TestCase):
    """Require the actual server and master to shut down without crashes."""

    def test_exit_codes(self, proc_info, master, server):
        """Check both launched processes, independently of assertion outcomes."""
        assertExitCodes(proc_info, process=master)
        assertExitCodes(proc_info, process=server)
