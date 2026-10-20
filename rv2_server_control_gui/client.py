"""Poll control-server services without blocking the GUI event loop."""

import dataclasses
import time

import r1_interfaces.msg
import r1_interfaces.srv
import rclpy.client
import rclpy.expand_topic_name
import rclpy.node
import rclpy.task
import rclpy.validate_full_topic_name


@dataclasses.dataclass
class GuiState:
    """Keep the latest server snapshot separate from local request progress."""

    status: r1_interfaces.msg.ControlServerStatus | None = None
    connected: bool = False
    connection_message: str = "Waiting for control server."
    last_update: float | None = None
    operation_pending: bool = False
    operation_message: str = ""
    operation_success: bool | None = None


@dataclasses.dataclass
class _Pending:
    client: rclpy.client.Client
    future: rclpy.task.Future
    deadline: float
    kind: str = ""


class ControlServerGuiNode(rclpy.node.Node):
    """Read status and submit one operation at a time on the GUI thread."""

    def __init__(self, *, clock=time.monotonic, **kwargs):
        """Create service clients; the caller drives the executor and tick."""
        super().__init__("control_server_gui", **kwargs)
        self._monotonic = clock
        self._state = GuiState()
        self._closed = False
        self._status_pending = None
        self._operation = None
        try:
            server = self.declare_parameter("server_name", "control_server").value
            if not isinstance(server, str) or not server.strip("/"):
                raise ValueError("server_name must be a nonempty ROS name")
            normalized = ("/" if server.startswith("/") else "") + "/".join(
                part for part in server.split("/") if part
            )
            self._target_prefix = rclpy.expand_topic_name.expand_topic_name(
                normalized, self.get_name(), self.get_namespace()
            )
            rclpy.validate_full_topic_name.validate_full_topic_name(self._target_prefix)
            self._poll_interval = self._milliseconds("poll_interval_ms", 500)
            self._timeout = self._milliseconds("service_timeout_ms", 2000)
            self._status_client = self.create_client(
                r1_interfaces.srv.ControlServerStatusReq,
                self._target_prefix + "/control_server_status",
            )
            self._switch_client = self.create_client(
                r1_interfaces.srv.SwitchControllerReq,
                self._target_prefix + "/switch_controller",
            )
            self._reset_client = self.create_client(
                r1_interfaces.srv.ResetInterruptReq,
                self._target_prefix + "/reset_interrupt",
            )
            self._next_poll = self._monotonic()
        except Exception:
            self.destroy_node()
            raise

    def _milliseconds(self, name, default):
        value = self.declare_parameter(name, default).value
        if type(value) is not int or value <= 0:
            raise ValueError(f"{name} must be a positive integer")
        return value / 1000.0

    @property
    def target_prefix(self):
        """Return the server prefix after expansion in this node's namespace."""
        return self._target_prefix

    @property
    def state(self):
        """Return state for read-only rendering on the same GUI thread."""
        return self._state

    def _disconnect(self, message):
        self._state.connected = False
        self._state.connection_message = message

    def _fresh(self, now):
        return (
            self._state.connected
            and self._state.last_update is not None
            and now - self._state.last_update < self._timeout
        )

    @staticmethod
    def _retire(pending):
        if pending is not None:
            pending.client.remove_pending_request(pending.future)
            if not pending.future.done():
                pending.future.cancel()

    def _cancel_status(self):
        self._retire(self._status_pending)
        self._status_pending = None

    @staticmethod
    def _response_message(response, labels):
        label = labels.get(
            response.response, f"Unknown response code {response.response}"
        )
        reason = response.reason.strip()
        return f"{label}: {reason}" if reason else label

    def _poll_status(self, now):
        self._next_poll = now + self._poll_interval
        try:
            if not self._status_client.service_is_ready():
                self._disconnect("Status service is unavailable; snapshot is stale.")
                return
            future = self._status_client.call_async(
                r1_interfaces.srv.ControlServerStatusReq.Request()
            )
            self._status_pending = _Pending(
                self._status_client, future, now + self._timeout
            )
        except Exception as error:
            self._disconnect(f"Status request failed: {error}")

    def _receive_status(self, pending, now):
        response_type = r1_interfaces.srv.ControlServerStatusReq.Response
        try:
            response = pending.future.result()
            if response.response != response_type.RESPONSE_SUCCESS:
                self._disconnect(
                    self._response_message(
                        response, {response_type.RESPONSE_ERROR: "Status query failed"}
                    )
                )
                return
            self._state.status = response.status
            self._state.last_update = now
            self._state.connected = True
            self._state.connection_message = "Connected."
        except Exception as error:
            self._disconnect(f"Status request failed: {error}")

    def _finish_operation(self, pending, now, *, timed_out=False):
        self._retire(pending)
        self._operation = None
        self._state.operation_pending = False
        self._state.operation_success = None
        if timed_out:
            self._state.operation_message = (
                "Request timed out; result unknown. Refresh status before "
                "retrying manually; the server may have applied the request."
            )
        else:
            try:
                response = pending.future.result()
                if pending.kind == "switch":
                    response_type = r1_interfaces.srv.SwitchControllerReq.Response
                    labels = {
                        response_type.RESPONSE_SUCCESS: "Switch accepted",
                        response_type.RESPONSE_NOT_FOUND: "Controller not registered",
                        response_type.RESPONSE_NOT_ACTIVE: "Controller not ACTIVE",
                        response_type.RESPONSE_UNSUPPORTED_TYPE: "Unsupported type",
                        response_type.RESPONSE_INTERRUPTED: (
                            "Wireless interrupt latched"
                        ),
                        response_type.RESPONSE_ERROR: "Switch failed",
                    }
                else:
                    response_type = r1_interfaces.srv.ResetInterruptReq.Response
                    labels = {
                        response_type.RESPONSE_SUCCESS: "Reset accepted",
                        response_type.RESPONSE_ERROR: "Reset failed",
                    }
                self._state.operation_success = (
                    response.response == response_type.RESPONSE_SUCCESS
                )
                self._state.operation_message = self._response_message(response, labels)
            except Exception as error:
                self._state.operation_message = (
                    f"Request failed; result unknown: {error}. "
                    "Refresh status before retrying manually."
                )
        self._cancel_status()
        self._disconnect("Refreshing status after operation; snapshot is stale.")
        self._next_poll = now

    def tick(self):
        """Process completed requests and deadlines without waiting for ROS."""
        if self._closed:
            return
        now = self._monotonic()
        if self._operation is not None:
            pending = self._operation
            if now >= pending.deadline:
                self._finish_operation(pending, now, timed_out=True)
            elif pending.future.done():
                self._finish_operation(pending, now)
            else:
                return
        if self._status_pending is not None:
            pending = self._status_pending
            if now >= pending.deadline:
                self._retire(pending)
                self._status_pending = None
                self._disconnect("Status request timed out; snapshot is stale.")
            elif pending.future.done():
                self._status_pending = None
                self._retire(pending)
                self._receive_status(pending, now)
        if self._state.connected and not self._fresh(now):
            self._disconnect("Status snapshot expired; waiting for a fresh reply.")
        if self._status_pending is None and now >= self._next_poll:
            self._poll_status(now)

    def _reject(self, message):
        self._state.operation_message = message
        self._state.operation_success = False
        return False

    def _can_operate(self):
        if self._closed:
            return self._reject("GUI client is closed.")
        if self._operation is not None:
            return False
        if not self._fresh(self._monotonic()):
            self._disconnect("A fresh status reply is required before an operation.")
            return self._reject("Wait for a fresh server status.")
        return True

    def _start_operation(self, kind, client, request):
        try:
            if not client.service_is_ready():
                self._disconnect("Operation service is unavailable.")
                return self._reject(
                    "Operation service is unavailable; request not sent."
                )
            future = client.call_async(request)
        except Exception as error:
            self._disconnect(f"Operation request could not be submitted: {error}")
            return self._reject(f"Operation request could not be submitted: {error}")
        self._cancel_status()
        self._operation = _Pending(
            client, future, self._monotonic() + self._timeout, kind
        )
        self._state.operation_pending = True
        self._state.operation_success = None
        self._state.operation_message = (
            "Waiting for switch response."
            if kind == "switch"
            else "Waiting for interrupt reset response."
        )
        self._disconnect("Operation pending; displayed snapshot is stale.")
        return True

    def switch_controller(self, name):
        """Submit a registered controller name; the server checks ACTIVE state."""
        if not self._can_operate():
            return False
        status = self._state.status
        if status.interrupted:
            return self._reject("Reset the wireless interrupt before switching.")
        if not name or not any(
            entry.controller_name == name for entry in status.registered_controllers
        ):
            return self._reject("Select a registered controller.")
        request = r1_interfaces.srv.SwitchControllerReq.Request()
        request.controller_name = name
        return self._start_operation("switch", self._switch_client, request)

    def reset_interrupt(self):
        """Submit an explicit reset only when the last fresh status is latched."""
        if not self._can_operate():
            return False
        if not self._state.status.interrupted:
            return self._reject("The wireless interrupt is not latched.")
        return self._start_operation(
            "reset", self._reset_client, r1_interfaces.srv.ResetInterruptReq.Request()
        )

    def close(self):
        """Cancel local waits without claiming that remote operations were undone."""
        if self._closed:
            return
        self._closed = True
        self._cancel_status()
        if self._operation is not None:
            self._retire(self._operation)
            self._operation = None
            self._state.operation_message = (
                "Client closed while waiting; operation result unknown."
            )
            self._state.operation_success = None
        self._state.operation_pending = False
        self._disconnect("GUI client closed.")
