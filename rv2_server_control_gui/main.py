"""Run Qt and asynchronous ROS service clients on one responsive event loop."""

import os
import signal
import sys

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
from rclpy import init, ok, try_shutdown
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from rv2_server_control_gui.client import ControlServerGuiNode
from rv2_server_control_gui.window import ControlServerWindow


def main(args=None):
    """Start the desktop client and release its ROS resources on window close."""
    if not any(
        os.environ.get(name)
        for name in ("DISPLAY", "WAYLAND_DISPLAY", "QT_QPA_PLATFORM")
    ):
        print(
            "control_server_gui requires a desktop display (DISPLAY or WAYLAND_DISPLAY). "
            "For headless testing use QT_QPA_PLATFORM=offscreen.",
            file=sys.stderr,
        )
        return 2
    app = QApplication([sys.argv[0]])
    app.setApplicationName("Control server")
    node = None
    executor = None
    timer = QTimer()
    old_handlers = {}
    exit_code = 0
    try:
        init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = ControlServerGuiNode()
        executor = SingleThreadedExecutor(context=node.context)
        executor.add_node(node)
        window = ControlServerWindow(node)

        def update():
            nonlocal exit_code
            if not ok(context=node.context):
                app.quit()
                return
            try:
                executor.spin_once(timeout_sec=0)
                node.tick()
                window.render()
            except ExternalShutdownException:
                app.quit()
            except Exception as error:
                node.get_logger().error(f"GUI event loop failed: {error}")
                exit_code = 1
                app.quit()

        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.signal(signum, lambda *_: app.quit())
        timer.timeout.connect(update)
        timer.start(20)
        window.show()
        app.exec_()
    except (RuntimeError, ValueError) as error:
        print(f"control_server_gui: {error}", file=sys.stderr)
        exit_code = 2
    finally:
        timer.stop()
        if node is not None:
            node.close()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        try_shutdown()
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
    return exit_code
