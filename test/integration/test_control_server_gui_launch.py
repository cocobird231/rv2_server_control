"""Exercise installed GUI launch namespacing and child-driven shutdown boundaries."""

import os
import pathlib
import signal
import sys
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import (
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch_testing import post_shutdown_test
from launch_testing.actions import ReadyToTest
from launch_testing.asserts import assertExitCodes
from launch_testing.util import KeepAliveProc
from r1_interfaces.srv import ControlServerStatusReq
from rclpy import create_node, init, shutdown
from rclpy.executors import SingleThreadedExecutor

SERVER, MASTER = "r1_gui_launch_server", "r1_gui_launch_master"
ROBOT_NAMESPACE = "/r1_gui_launch_robot"
OBSERVER_NAMESPACE = "/r1_gui_launch_observer"
STATUS_SERVICE = f"{ROBOT_NAMESPACE}/{SERVER}/control_server_status"
EXECUTABLES = ("control_server_gui", "control_server", "csm_master_node")


def executable_name(command):
    """Recognize only the actual installed executables, including Python scripts."""
    return next(
        (
            pathlib.Path(argument).name
            for argument in command
            if pathlib.Path(argument).name in EXECUTABLES
        ),
        None,
    )


def run_installed_scenario():
    """Run both includes in one isolated launch session and check every child exit."""
    launch_file = str(
        pathlib.Path(get_package_share_directory("rv2_server_control"))
        / "launch/control_server_gui.launch.py"
    )
    exited = []

    def record_exit(event, _context):
        name = executable_name(event.cmd)
        if name is not None:
            exited.append((name, event.returncode))

    def include(namespace, arguments):
        return GroupAction(
            actions=[
                PushRosNamespace(namespace),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(launch_file),
                    launch_arguments=arguments.items(),
                ),
            ]
        )

    launch = LaunchService(argv=[])
    launch.include_launch_description(
        LaunchDescription(
            [
                RegisterEventHandler(OnProcessExit(on_exit=record_exit)),
                include(
                    ROBOT_NAMESPACE,
                    {
                        "start_server": "true",
                        "start_master": "true",
                        "server_name": SERVER,
                        "master_name": MASTER,
                        "poll_interval_ms": "50",
                        "service_timeout_ms": "300",
                    },
                ),
                include(
                    OBSERVER_NAMESPACE,
                    {
                        "server_name": f"{ROBOT_NAMESPACE}/{SERVER}",
                        "poll_interval_ms": "50",
                        "service_timeout_ms": "300",
                    },
                ),
            ]
        )
    )
    result = launch.run()
    expected = {"control_server_gui": 2, "control_server": 1, "csm_master_node": 1}
    counts = {name: sum(child == name for child, _ in exited) for name in expected}
    if counts != expected or any(code != 0 for _, code in exited):
        print(f"Unexpected installed child exits: {exited}", file=sys.stderr)
        return 1
    return result


def generate_test_description():
    """Keep combo Shutdown events inside a process owned by launch_testing."""
    process = ExecuteProcess(
        cmd=[
            sys.executable,
            str(pathlib.Path(__file__).resolve()),
            "--launch-scenario",
        ],
        additional_env={"QT_QPA_PLATFORM": "offscreen"},
        output="screen",
    )
    # The scenario intentionally exits before graph/process assertions finish.
    # Keep the outer harness alive until launch_testing completes those checks.
    return LaunchDescription([KeepAliveProc(), process, ReadyToTest()]), {
        "scenario": process
    }


def descendant_commands(parent_pid):
    """Inspect only descendants of this test's recorded launch process."""
    pending = [parent_pid]
    commands = {}
    while pending:
        pid = pending.pop()
        try:
            children = (
                pathlib.Path(f"/proc/{pid}/task/{pid}/children").read_text().split()
            )
        except OSError:
            continue
        for child_text in children:
            child = int(child_text)
            if child in commands:
                continue
            try:
                command = (
                    pathlib.Path(f"/proc/{child}/cmdline").read_bytes().split(b"\0")
                )
            except OSError:
                continue
            commands[child] = [part.decode() for part in command if part]
            pending.append(child)
    return commands


def find_child(parent_pid, executable, namespace):
    """Match an exact executable and namespace before targeting a child PID."""
    matches = [
        pid
        for pid, command in descendant_commands(parent_pid).items()
        if executable_name(command) == executable and f"__ns:={namespace}" in command
    ]
    return matches[0] if len(matches) == 1 else None


class TestInstalledGuiLaunch(unittest.TestCase):
    """Check installed clients, relative namespaces and exact process lifetimes."""

    @classmethod
    def setUpClass(cls):
        """Create only a service observer; this process is not a control source."""
        init()
        cls.node = create_node("r1_gui_launch_probe")
        cls.client = cls.node.create_client(ControlServerStatusReq, STATUS_SERVICE)
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)

    @classmethod
    def tearDownClass(cls):
        """Release the observer after the installed child processes have exited."""
        cls.executor.shutdown()
        cls.node.destroy_node()
        shutdown()

    def wait_for(self, predicate, timeout=20.0):
        """Allow discovery and responses while checking a concrete predicate."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return True
        return predicate()

    def query_status(self):
        """Require a new actual server response rather than stale graph entries."""
        future = self.client.call_async(ControlServerStatusReq.Request())
        self.assertTrue(self.wait_for(future.done))
        self.assertIsNone(future.exception())
        response = future.result()
        self.assertEqual(response.response, response.RESPONSE_SUCCESS, response.reason)
        self.assertEqual(response.status.server_name, SERVER)
        self.assertFalse(response.status.interrupted)
        self.assertFalse(response.status.has_output)
        self.assertEqual(list(response.status.registered_controllers), [])
        return response.status

    def test_gui_only_and_combo_child_exit_boundaries(self, proc_info, scenario):
        """Close each actual GUI child and observe its distinct cleanup contract."""
        proc_info.assertWaitForStartup(process=scenario, timeout=10.0)
        parent_pid = proc_info[scenario].pid
        self.assertTrue(self.client.wait_for_service(timeout_sec=20.0))
        self.query_status()
        self.assertTrue(
            self.wait_for(
                lambda: (
                    self.node.count_clients(STATUS_SERVICE) == 3
                    and ("control_server_gui", ROBOT_NAMESPACE)
                    in self.node.get_node_names_and_namespaces()
                    and ("control_server_gui", OBSERVER_NAMESPACE)
                    in self.node.get_node_names_and_namespaces()
                )
            ),
            "both installed GUIs must target the namespaced server",
        )
        self.assertEqual(
            self.node.count_services(f"{ROBOT_NAMESPACE}/{MASTER}/register"), 1
        )
        self.assertEqual(self.node.count_services("/csm_master/register"), 0)
        self.assertEqual(
            self.node.count_services(f"/{SERVER}/control_server_status"), 0
        )
        self.assertEqual(self.node.get_node_names().count("csm_master_node"), 1)
        children = {
            "observer": find_child(
                parent_pid, "control_server_gui", OBSERVER_NAMESPACE
            ),
            "gui": find_child(parent_pid, "control_server_gui", ROBOT_NAMESPACE),
            "server": find_child(parent_pid, "control_server", ROBOT_NAMESPACE),
            "master": find_child(parent_pid, "csm_master_node", ROBOT_NAMESPACE),
        }
        self.assertTrue(
            all(children.values()), f"exact installed child PIDs missing: {children}"
        )
        self.assertEqual(len(set(children.values())), 4)

        # The GUI-only include shares this launch session with an external server.
        # Its exit must not emit a global Shutdown event or stop that server.
        os.kill(children["observer"], signal.SIGTERM)
        self.assertTrue(
            self.wait_for(
                lambda: not pathlib.Path(f"/proc/{children['observer']}").exists()
            )
        )
        self.assertTrue(
            self.wait_for(lambda: self.node.count_clients(STATUS_SERVICE) == 2)
        )
        # Allow an erroneous global Shutdown to propagate before proving the
        # original server still answers, rather than winning that teardown race.
        settle_until = time.monotonic() + 0.5
        self.assertTrue(self.wait_for(lambda: time.monotonic() >= settle_until))
        self.assertTrue(pathlib.Path(f"/proc/{parent_pid}").exists())
        self.assertEqual(
            find_child(parent_pid, "control_server", ROBOT_NAMESPACE),
            children["server"],
        )
        self.assertEqual(
            find_child(parent_pid, "csm_master_node", ROBOT_NAMESPACE),
            children["master"],
        )
        self.query_status()

        # Only the GUI is signalled: its installed OnProcessExit handler must
        # terminate the combo's server/master and let the nested launch finish.
        os.kill(children["gui"], signal.SIGINT)
        proc_info.assertWaitForShutdown(process=scenario, timeout=20.0)
        self.assertEqual(proc_info[scenario].returncode, 0)
        self.assertTrue(
            self.wait_for(
                lambda: all(
                    not pathlib.Path(f"/proc/{pid}").exists()
                    for pid in children.values()
                )
            ),
            "a GUI-owned server or master survived combo shutdown",
        )
        self.assertTrue(
            self.wait_for(lambda: self.node.count_services(STATUS_SERVICE) == 0)
        )


@post_shutdown_test()
class TestInstalledGuiShutdown(unittest.TestCase):
    """The nested launcher checks each actual GUI, server and master exit code."""

    def test_exit_codes(self, proc_info):
        """Reject hidden child crashes as well as an abnormal launcher shutdown."""
        assertExitCodes(proc_info)


if __name__ == "__main__":
    if sys.argv[1:] != ["--launch-scenario"]:
        raise SystemExit("Use this file through launch_testing.")
    raise SystemExit(run_installed_scenario())
