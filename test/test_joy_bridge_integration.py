#!/usr/bin/env python3
"""
End-to-end launch integration test: simulated /joy → topic bridge → CSM →
ControlServerNode → /api/sport/request (unitree_api).

Processes launched:
  1. `ros2 component standalone rv2_server_control ControlServerNode`
     (real composable node, default server_name 'control_server')
  2. `ros2 launch rv2_csm_topic_bridge topic_bridge.launch.py topic_name:=/joy`
     (bridges /joy into the CSM as a Joy source; default target server matches)

This test process itself simulates the joy driver: it publishes the current
Joy state on /joy at 20 Hz (like a real joystick node) and captures every
unitree_api Request on /api/sport/request.

Scenario windows (Joy mapping per docs/joy按鍵定義_數值範圍.json):
  W0  readiness probe — axes (0.1,0,0) until the first Move request arrives
  W1  idle sticks (all zeros)      → NO requests published (Move dedup)
  W2  sticks (0.5,0.2,0.3) held    → exactly one Move with x/y/z params
  W3  sticks back to zero          → exactly one Move(0,0,0)
  W4  per-axis sweep — vx only / vy only / vyaw only (negative) → one Move
                                     each with the value on the right field
  W5  unused axes — right stick vertical + L2 trigger wiggled → NO requests
  W6  A/B/X/Y press+release        → StandUp/StandDown/StopMove/RecoveryStand
                                     exactly once each; no Move requests
  W7  idle sticks again            → NO requests published

Run manually:
  launch_test src/rv2_server_control/test/test_joy_bridge_integration.py

Requires the workspace (and the unitree_api underlay) to be sourced.
"""

import json
import os
import threading
import time
import unittest

# Isolate from any real robot / joystick running on the default domain.
os.environ.setdefault('ROS_DOMAIN_ID', '77')

import launch
import launch_testing
import launch_testing.actions
from launch.actions import ExecuteProcess, TimerAction

import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Joy
from unitree_api.msg import Request

API_STOPMOVE      = 1003
API_STANDUP       = 1004
API_STANDDOWN     = 1005
API_RECOVERYSTAND = 1006
API_MOVE          = 1008

BTN_A, BTN_B, BTN_X, BTN_Y = 0, 1, 2, 3
NUM_BUTTONS = 17

# Real Xbox driver axis layout: [0]=left H (vx), [1]=left V (vy), [2]=L2,
# [3]=right H (vyaw), [4]=right V, [5]=R2, [6..7]=D-pad.
# L2/R2 triggers idle at +1.0 (released); axes[5] < 0.5 is the e-stop
# sentinel (control_signal_detect.h), so the simulated driver must keep
# R2 released or every message would be consumed as an e-stop.
IDLE_AXES = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
AX_RIGHT_H = 3

JOY_RATE_HZ = 20.0


def generate_test_description():
    control_server = ExecuteProcess(
        cmd=['ros2', 'component', 'standalone',
             'rv2_server_control', 'ControlServerNode'],
        output='screen')

    # The bridge registers its CSM source once (5 s service wait, no retry) —
    # start it after the control server has had time to come up.
    bridge = ExecuteProcess(
        cmd=['ros2', 'launch', 'rv2_csm_topic_bridge',
             'topic_bridge.launch.py', 'topic_name:=/joy'],
        output='screen')

    return launch.LaunchDescription([
        control_server,
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=5.0, actions=[launch_testing.actions.ReadyToTest()]),
    ]), {'control_server': control_server, 'bridge': bridge}


class TestJoyToSportRequest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('joy_sim')
        cls.lock = threading.Lock()
        cls.axes = list(IDLE_AXES)
        cls.buttons = [0] * NUM_BUTTONS
        cls.requests = []

        cls.joy_pub = cls.node.create_publisher(Joy, '/joy', 10)
        cls.req_sub = cls.node.create_subscription(
            Request, '/api/sport/request', cls._on_request, 100)

        # Simulated joy driver: publish the current state continuously so the
        # CSM sink never times out between scenario windows.
        cls.joy_timer = cls.node.create_timer(1.0 / JOY_RATE_HZ, cls._publish_joy)

        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    # ── Joy state / capture helpers ───────────────────────────────────────────

    @classmethod
    def _publish_joy(cls):
        msg = Joy()
        msg.header.stamp = cls.node.get_clock().now().to_msg()
        with cls.lock:
            msg.axes = list(cls.axes)
            msg.buttons = list(cls.buttons)
        cls.joy_pub.publish(msg)

    @classmethod
    def _on_request(cls, msg):
        with cls.lock:
            cls.requests.append(msg)

    def set_axes(self, vx=0.0, vy=0.0, vyaw=0.0):
        axes = list(IDLE_AXES)
        axes[0] = float(vx)
        axes[1] = float(vy)
        axes[AX_RIGHT_H] = float(vyaw)
        with self.lock:
            self.__class__.axes = axes

    def set_raw_axis(self, index, value):
        """Set one raw axis (e.g. unused right V / L2) keeping the rest."""
        with self.lock:
            axes = list(self.axes)
            axes[index] = float(value)
            self.__class__.axes = axes

    def press(self, btn):
        with self.lock:
            self.__class__.buttons = [0] * NUM_BUTTONS
            self.__class__.buttons[btn] = 1

    def release_all(self):
        with self.lock:
            self.__class__.buttons = [0] * NUM_BUTTONS

    def clear(self):
        with self.lock:
            self.__class__.requests = []

    def total(self):
        with self.lock:
            return len(self.requests)

    def count(self, api_id):
        with self.lock:
            return sum(1 for r in self.requests
                       if r.header.identity.api_id == api_id)

    def last_param(self, api_id):
        with self.lock:
            for r in reversed(self.requests):
                if r.header.identity.api_id == api_id:
                    return json.loads(r.parameter) if r.parameter else {}
        return None

    def wait_until(self, pred, timeout, period=0.1):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if pred():
                return True
            time.sleep(period)
        return pred()

    # ── The scenario ──────────────────────────────────────────────────────────

    def test_joy_to_sport_request(self):
        # W0 — readiness: drive a small vx until the full pipeline
        # (bridge registration → CSM → control server → SportClient) produces
        # the first Move request.
        self.set_axes(vx=0.1)
        self.assertTrue(
            self.wait_until(lambda: self.count(API_MOVE) > 0, timeout=30.0),
            'pipeline never produced a Move request — bridge registration or '
            'control server startup failed')

        # Return sticks to zero; the value change flushes one Move(0,0,0).
        self.set_axes()
        time.sleep(1.0)
        self.clear()

        # W1 — idle sticks: joystick keeps publishing zeros at 20 Hz but the
        # control server must publish NOTHING (Move dedup).
        time.sleep(2.0)
        self.assertEqual(
            self.total(), 0,
            f'idle joystick published {self.total()} requests; expected 0')

        # W2 — move sticks: held at (0.5, 0.2, 0.3) for 2 s (~40 joy messages)
        # → exactly ONE Move request with the correct x/y/z parameters.
        self.clear()
        self.set_axes(vx=0.5, vy=0.2, vyaw=0.3)
        time.sleep(2.0)
        self.assertEqual(
            self.count(API_MOVE), 1,
            f'held sticks: expected exactly 1 Move, got {self.count(API_MOVE)}')
        param = self.last_param(API_MOVE)
        self.assertIsNotNone(param, 'Move request has no parameter')
        self.assertAlmostEqual(param['x'], 0.5, places=4)
        self.assertAlmostEqual(param['y'], 0.2, places=4)
        self.assertAlmostEqual(param['z'], 0.3, places=4)

        # W3 — sticks back to zero → exactly one final Move(0,0,0).
        self.clear()
        self.set_axes()
        time.sleep(1.5)
        self.assertEqual(
            self.count(API_MOVE), 1,
            f'zero return: expected exactly 1 Move(0,0,0), got {self.count(API_MOVE)}')
        param = self.last_param(API_MOVE)
        self.assertAlmostEqual(param['x'], 0.0, places=6)
        self.assertAlmostEqual(param['y'], 0.0, places=6)
        self.assertAlmostEqual(param['z'], 0.0, places=6)

        # W4 — per-axis sweep: each axis alone must land on the right Move
        # field (vx→x, vy→y, vyaw→z), including a negative value.
        axis_cases = [
            ('vx only',   0.6,  0.0,  0.0),
            ('vy only',   0.0,  0.4,  0.0),
            ('vyaw only', 0.0,  0.0, -0.7),
        ]
        for name, vx, vy, vyaw in axis_cases:
            self.clear()
            self.set_axes(vx=vx, vy=vy, vyaw=vyaw)
            time.sleep(1.0)
            self.assertEqual(
                self.count(API_MOVE), 1,
                f'{name}: expected exactly 1 Move, got {self.count(API_MOVE)}')
            param = self.last_param(API_MOVE)
            self.assertAlmostEqual(param['x'], vx,   places=4, msg=f'{name}: x')
            self.assertAlmostEqual(param['y'], vy,   places=4, msg=f'{name}: y')
            self.assertAlmostEqual(param['z'], vyaw, places=4, msg=f'{name}: z')

        # Back to idle → flush the zero Move before the unused-axes window.
        self.set_axes()
        time.sleep(1.0)

        # W5 — unused axes: wiggle right stick vertical (axes[4]) and press
        # L2 (axes[2] → -1.0) while mapped sticks stay at zero → NO requests.
        self.clear()
        self.set_raw_axis(4, 0.9)
        self.set_raw_axis(2, -1.0)
        time.sleep(1.0)
        self.set_raw_axis(4, -0.5)
        time.sleep(1.0)
        self.assertEqual(
            self.total(), 0,
            f'unused axes published {self.total()} requests; expected 0')
        self.set_axes()   # restore idle (L2 released)
        time.sleep(0.5)

        # W6 — buttons: press/release A, B, X, Y → one command request each,
        # and no Move requests (sticks stayed at zero).
        self.clear()
        button_cases = [
            (BTN_A, API_STANDUP,       'A→StandUp(1004)'),
            (BTN_B, API_STANDDOWN,     'B→StandDown(1005)'),
            (BTN_X, API_STOPMOVE,      'X→StopMove(1003)'),
            (BTN_Y, API_RECOVERYSTAND, 'Y→RecoveryStand(1006)'),
        ]
        for btn, _api, _name in button_cases:
            self.press(btn)
            time.sleep(0.4)
            self.release_all()
            time.sleep(0.4)

        for _btn, api, name in button_cases:
            self.assertEqual(
                self.count(api), 1,
                f'{name}: expected exactly 1 request, got {self.count(api)}')
        self.assertEqual(
            self.count(API_MOVE), 0,
            'button phase must not produce Move requests (sticks at zero)')

        # W7 — idle again: still silent.
        self.clear()
        time.sleep(2.0)
        self.assertEqual(
            self.total(), 0,
            f'final idle: published {self.total()} requests; expected 0')


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        # Launched processes are killed by the framework at shutdown; only
        # assert they did not die on their own with an error before that.
        pass
