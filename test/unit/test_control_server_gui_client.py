"""Exercise GUI service sequencing and stale-state protection without ROS nodes."""

import dataclasses
import types

import pytest
import r1_interfaces.msg
import r1_interfaces.srv
import rclpy.exceptions
import rclpy.node
from rv2_server_control_gui import client


class FakeClock:
    """Provide monotonic time without sleeping."""

    def __init__(self):
        self.now = 10.0

    def __call__(self):
        """Return the current simulated monotonic time."""
        return self.now

    def advance(self, seconds):
        """Advance time explicitly at a request boundary."""
        self.now += seconds


class FakeFuture:
    """Allow a delayed transport reply even after local cancellation."""

    def __init__(self):
        self.complete = False
        self.cancelled = False
        self.value = None
        self.error = None
        self.result_reads = 0

    def done(self):
        """Report a completed reply or local cancellation."""
        return self.complete or self.cancelled

    def result(self):
        """Return a reply and expose accidental consumption of retired replies."""
        self.result_reads += 1
        if self.error:
            raise self.error
        return self.value

    def cancel(self):
        """Cancel the local wait without pretending to undo a server operation."""
        self.cancelled = True

    def respond(self, value=None, error=None):
        """Deliver an independently scheduled server reply."""
        self.value = value
        self.error = error
        self.complete = True


class FakeClient:
    """Record requests and provide independently controlled service readiness."""

    def __init__(self, service, name):
        self.service = service
        self.name = name
        self.ready = True
        self.requests = []
        self.futures = []
        self.retired = []
        self.call_error = None

    def service_is_ready(self):
        """Check readiness without waiting."""
        return self.ready

    def call_async(self, request):
        """Record a real generated request message for later completion."""
        if self.call_error:
            raise self.call_error
        assert isinstance(request, self.service.Request)
        self.requests.append(request)
        future = FakeFuture()
        self.futures.append(future)
        return future

    def remove_pending_request(self, future):
        """Record retirement independently from local future cancellation."""
        self.retired.append(future)


def status_message(*, active="joy", interrupted=False, names=("joy", "twist")):
    """Construct actual interface messages with independently chosen truth."""
    return r1_interfaces.msg.ControlServerStatus(
        server_name="control_server",
        active_controller_name=active,
        interrupted=interrupted,
        registered_controllers=[
            r1_interfaces.msg.ControlSignalInfo(controller_name=name) for name in names
        ],
    )


@dataclasses.dataclass
class Harness:
    """Bundle the public node API with controlled service peers."""

    node: client.ControlServerGuiNode
    clock: FakeClock
    status: FakeClient
    switch: FakeClient
    reset: FakeClient
    ros_clock: object

    def reply_status(self, message=None, *, response=0, reason=""):
        """Complete the latest status request, then run one GUI tick."""
        if message is None:
            message = status_message()
        self.status.futures[-1].respond(
            r1_interfaces.srv.ControlServerStatusReq.Response(
                response=response, reason=reason, status=message
            )
        )
        self.node.tick()
        return message

    def connect(self, message=None):
        """Observe an actual status response before enabling operations."""
        self.node.tick()
        return self.reply_status(message)


@pytest.fixture
def make_node(monkeypatch):
    """Replace only ROS transport; retain generated messages and client policy."""
    created = []
    options = {}
    services = {}
    ros_clock = object()

    def initialize_node(self, name, **kwargs):
        self._clock = ros_clock
        self._test_name = name
        self._test_namespace = kwargs.get("namespace", "/")

    monkeypatch.setattr(rclpy.node.Node, "__init__", initialize_node)
    monkeypatch.setattr(rclpy.node.Node, "destroy_node", lambda self: None)
    monkeypatch.setattr(rclpy.node.Node, "get_name", lambda self: self._test_name)
    monkeypatch.setattr(
        rclpy.node.Node, "get_namespace", lambda self: self._test_namespace
    )
    monkeypatch.setattr(
        rclpy.node.Node,
        "declare_parameter",
        lambda self, name, default: types.SimpleNamespace(
            value=options.get(name, default)
        ),
    )

    def create_client(self, service, name):
        peer = FakeClient(service, name)
        services[name.rsplit("/", 1)[-1]] = peer
        return peer

    monkeypatch.setattr(rclpy.node.Node, "create_client", create_client)

    def make(parameters=None, **kwargs):
        options.clear()
        options.update(parameters or {})
        services.clear()
        clock = FakeClock()
        node = client.ControlServerGuiNode(clock=clock, **kwargs)
        created.append(node)
        return Harness(
            node,
            clock,
            services["control_server_status"],
            services["switch_controller"],
            services["reset_interrupt"],
            ros_clock,
        )

    yield make
    for node in created:
        node.close()


def test_initial_state_and_absolute_service_names(make_node):
    """Namespaced servers use exactly their absolute operational endpoints."""
    harness = make_node(
        {"server_name": "//robot//control_server///"}, namespace="/other_robot"
    )
    assert harness.node.target_prefix == "/robot/control_server"
    assert [
        harness.status.name,
        harness.switch.name,
        harness.reset.name,
    ] == [
        "/robot/control_server/control_server_status",
        "/robot/control_server/switch_controller",
        "/robot/control_server/reset_interrupt",
    ]
    state = harness.node.state
    assert state.status is None
    assert not state.connected
    assert state.last_update is None
    assert not state.operation_pending
    assert not harness.node.switch_controller("joy")
    assert not harness.node.reset_interrupt()
    assert not harness.switch.requests and not harness.reset.requests
    assert harness.node.get_clock() is harness.ros_clock


@pytest.mark.parametrize(
    "server_name,namespace,expected",
    [
        ("control_server", "/", "/control_server"),
        ("control_server", "/robot", "/robot/control_server"),
        ("nested//control_server/", "/robot", "/robot/nested/control_server"),
        ("/control_server", "/robot", "/control_server"),
    ],
)
def test_service_prefix_respects_relative_and_absolute_names(
    make_node, server_name, namespace, expected
):
    """Resolve relative targets in the node namespace and keep absolute targets."""
    harness = make_node({"server_name": server_name}, namespace=namespace)
    assert harness.node.target_prefix == expected
    assert harness.status.name == expected + "/control_server_status"
    assert harness.switch.name == expected + "/switch_controller"
    assert harness.reset.name == expected + "/reset_interrupt"


@pytest.mark.parametrize(
    "parameters",
    [
        {"server_name": ""},
        {"server_name": "///"},
        {"server_name": "has spaces"},
        {"server_name": 3},
        {"poll_interval_ms": 0},
        {"poll_interval_ms": -1},
        {"poll_interval_ms": True},
        {"poll_interval_ms": 0.5},
        {"service_timeout_ms": 0},
        {"service_timeout_ms": -1},
        {"service_timeout_ms": False},
        {"service_timeout_ms": "2000"},
    ],
)
def test_invalid_parameters_fail_before_requests(make_node, parameters):
    """Reject invalid names and nonpositive or noninteger timing settings."""
    with pytest.raises((ValueError, rclpy.exceptions.InvalidTopicNameException)):
        make_node(parameters)


def test_polling_is_bounded_and_recovers_when_service_appears(make_node):
    """Keep at most one status request and honor the configured poll interval."""
    harness = make_node()
    harness.status.ready = False
    harness.node.tick()
    assert not harness.status.requests
    assert not harness.node.state.connected
    harness.status.ready = True
    harness.clock.advance(0.49)
    harness.node.tick()
    assert not harness.status.requests
    harness.clock.advance(0.02)
    harness.node.tick()
    assert len(harness.status.requests) == 1
    harness.clock.advance(1.0)
    for _ in range(5):
        harness.node.tick()
    assert len(harness.status.requests) == 1
    message = harness.reply_status()
    assert harness.node.state.status == message
    assert harness.node.state.connected
    assert harness.node.state.last_update == harness.clock()
    assert len(harness.status.requests) == 2
    for _ in range(5):
        harness.node.tick()
    assert len(harness.status.requests) == 2


def test_status_timeout_preserves_snapshot_and_ignores_late_reply(make_node):
    """An expired reply cannot replace retained state or a later recovered state."""
    harness = make_node()
    original = harness.connect()
    updated = harness.node.state.last_update
    harness.clock.advance(0.5)
    harness.node.tick()
    late = harness.status.futures[-1]
    harness.clock.advance(2.0)
    harness.node.tick()
    assert late.cancelled
    assert late in harness.status.retired
    assert not harness.node.state.connected
    assert harness.node.state.status == original
    assert harness.node.state.last_update == updated
    assert not harness.node.switch_controller("twist")
    late.respond(
        r1_interfaces.srv.ControlServerStatusReq.Response(
            status=status_message(active="obsolete")
        )
    )
    harness.node.tick()
    assert late.result_reads == 0
    assert harness.node.state.status == original
    recovered = harness.reply_status(status_message(active="twist"))
    assert harness.node.state.connected
    assert harness.node.state.status == recovered
    assert harness.node.state.last_update > updated


def test_success_arriving_at_deadline_is_not_accepted(make_node):
    """A reply processed at the timeout boundary is already stale."""
    harness = make_node()
    harness.node.tick()
    expired = harness.status.futures[-1]
    expired.respond(r1_interfaces.srv.ControlServerStatusReq.Response())
    harness.clock.advance(2.0)
    harness.node.tick()
    assert expired.result_reads == 0
    assert not harness.node.state.connected


@pytest.mark.parametrize("code,reason", [(1, ""), (1, "server error"), (99, "")])
def test_status_error_codes_do_not_replace_good_data(make_node, code, reason):
    """Typed failures and unknown codes retain the last successful snapshot."""
    harness = make_node()
    original = harness.connect()
    harness.clock.advance(0.5)
    harness.node.tick()
    harness.reply_status(
        status_message(active="must not appear"), response=code, reason=reason
    )
    assert not harness.node.state.connected
    assert harness.node.state.status == original
    if reason:
        assert reason in harness.node.state.connection_message
    else:
        assert str(code) in harness.node.state.connection_message or (
            "failed" in harness.node.state.connection_message
        )


def test_expired_snapshot_cannot_authorize_actions_between_ticks(make_node):
    """A click checks monotonic freshness even if the Qt tick was delayed."""
    harness = make_node({"poll_interval_ms": 5000})
    harness.connect()
    harness.clock.advance(2.0)
    assert not harness.node.switch_controller("twist")
    assert not harness.switch.requests
    assert not harness.node.state.connected


def test_switch_waits_for_post_operation_truth_and_rejects_duplicates(make_node):
    """A successful switch reply never substitutes for fresh server state."""
    harness = make_node()
    original = harness.connect()
    harness.clock.advance(0.5)
    harness.node.tick()
    stale = harness.status.futures[-1]
    assert harness.node.switch_controller("twist")
    assert harness.switch.requests[0].controller_name == "twist"
    assert harness.node.state.operation_pending
    assert not harness.node.state.connected
    assert harness.node.state.status == original
    assert stale.cancelled and stale in harness.status.retired
    assert not harness.node.switch_controller("joy")
    assert not harness.node.reset_interrupt()
    assert len(harness.switch.requests) == 1
    stale.respond(
        r1_interfaces.srv.ControlServerStatusReq.Response(
            status=status_message(active="stale")
        )
    )
    harness.node.tick()
    assert stale.result_reads == 0
    assert len(harness.status.requests) == 2
    harness.switch.futures[0].respond(r1_interfaces.srv.SwitchControllerReq.Response())
    harness.node.tick()
    assert not harness.node.state.operation_pending
    assert harness.node.state.operation_success is True
    assert not harness.node.state.connected
    assert harness.node.state.status == original
    assert len(harness.status.requests) == 3
    assert not harness.node.switch_controller("joy")
    truth = harness.reply_status(status_message(active="joy", interrupted=True))
    assert harness.node.state.connected
    assert harness.node.state.status == truth
    assert truth.active_controller_name != "twist"


@pytest.mark.parametrize(
    "code,label",
    [
        (1, "not registered"),
        (2, "not ACTIVE"),
        (3, "Unsupported"),
        (4, "interrupt"),
        (5, "failed"),
        (99, "99"),
    ],
)
def test_switch_failure_labels_and_server_reasons(make_node, code, label):
    """Keep typed rejection labels when the server has no diagnostic reason."""
    harness = make_node()
    harness.connect()
    assert harness.node.switch_controller("twist")
    harness.switch.futures[-1].respond(
        r1_interfaces.srv.SwitchControllerReq.Response(response=code)
    )
    harness.node.tick()
    assert harness.node.state.operation_success is False
    assert label in harness.node.state.operation_message
    assert not harness.node.state.connected
    assert len(harness.status.requests) == 2
    harness.reply_status()
    assert harness.node.switch_controller("twist")
    harness.switch.futures[-1].respond(
        r1_interfaces.srv.SwitchControllerReq.Response(
            response=code, reason="precise server diagnostic"
        )
    )
    harness.node.tick()
    assert "precise server diagnostic" in harness.node.state.operation_message
    assert label in harness.node.state.operation_message


def test_switch_and_reset_guard_registration_and_interrupt_state(make_node):
    """The GUI cannot request an unregistered controller or an unnecessary reset."""
    harness = make_node()
    harness.connect()
    assert not harness.node.switch_controller("")
    assert not harness.node.switch_controller("missing")
    assert not harness.node.reset_interrupt()
    assert not harness.switch.requests and not harness.reset.requests
    harness.clock.advance(0.5)
    harness.node.tick()
    latched = harness.reply_status(status_message(active="", interrupted=True))
    assert not harness.node.switch_controller("twist")
    assert harness.node.reset_interrupt()
    assert harness.node.state.operation_pending
    assert not harness.node.reset_interrupt()
    assert len(harness.reset.requests) == 1
    harness.reset.futures[-1].respond(r1_interfaces.srv.ResetInterruptReq.Response())
    harness.node.tick()
    assert harness.node.state.operation_success is True
    assert harness.node.state.status == latched
    assert harness.node.state.status.interrupted
    assert not harness.node.state.connected
    harness.reply_status(status_message(active="", interrupted=False))
    assert not harness.node.state.status.interrupted
    assert not harness.node.reset_interrupt()


@pytest.mark.parametrize("code,label", [(1, "Reset failed"), (99, "99")])
def test_reset_failure_keeps_latch_until_status_confirms(make_node, code, label):
    """A rejected reset cannot clear the displayed interrupt latch."""
    harness = make_node()
    latched = harness.connect(status_message(active="", interrupted=True))
    assert harness.node.reset_interrupt()
    harness.reset.futures[-1].respond(
        r1_interfaces.srv.ResetInterruptReq.Response(
            response=code, reason="reset diagnostic"
        )
    )
    harness.node.tick()
    assert harness.node.state.operation_success is False
    assert label in harness.node.state.operation_message
    assert "reset diagnostic" in harness.node.state.operation_message
    assert harness.node.state.status == latched
    assert harness.node.state.status.interrupted
    assert not harness.node.state.connected
    assert len(harness.status.requests) == 2
    harness.reply_status(latched)
    assert harness.node.state.connected
    assert harness.node.state.status.interrupted


@pytest.mark.parametrize("operation", ["switch", "reset"])
def test_operation_timeout_is_unknown_and_never_retried(make_node, operation):
    """A timed-out request may execute remotely; only later status is authoritative."""
    harness = make_node()
    harness.connect(status_message(interrupted=operation == "reset"))
    peer = getattr(harness, operation)

    def invoke():
        if operation == "switch":
            return harness.node.switch_controller("twist")
        return harness.node.reset_interrupt()

    assert invoke()
    expired = peer.futures[-1]
    harness.clock.advance(2.0)
    harness.node.tick()
    assert expired.cancelled and expired in peer.retired
    assert not harness.node.state.operation_pending
    assert harness.node.state.operation_success is None
    assert "unknown" in harness.node.state.operation_message
    assert "manually" in harness.node.state.operation_message
    assert not harness.node.state.connected
    expired.respond(peer.service.Response())
    harness.node.tick()
    assert expired.result_reads == 0
    harness.reply_status(status_message(active="twist"))
    harness.clock.advance(0.5)
    harness.node.tick()
    harness.reply_status(status_message(active="twist"))
    assert len(peer.requests) == 1
    assert harness.node.state.connected
    assert "unknown" in harness.node.state.operation_message


@pytest.mark.parametrize("operation", ["switch", "reset"])
def test_unavailable_operation_service_does_not_send(make_node, operation):
    """Discovery is nonblocking, and no request is queued for automatic retry."""
    harness = make_node()
    harness.connect(status_message(interrupted=operation == "reset"))
    peer = getattr(harness, operation)
    peer.ready = False
    accepted = (
        harness.node.switch_controller("twist")
        if operation == "switch"
        else harness.node.reset_interrupt()
    )
    assert not accepted
    assert not peer.requests
    assert not harness.node.state.operation_pending
    assert "not sent" in harness.node.state.operation_message
    peer.ready = True
    harness.clock.advance(0.5)
    harness.node.tick()
    harness.reply_status()
    assert not peer.requests


def test_transport_errors_recover_without_losing_last_good_state(make_node):
    """Future exceptions and submit failures stay inside the event loop."""
    harness = make_node()
    original = harness.connect()
    harness.clock.advance(0.5)
    harness.node.tick()
    harness.status.futures[-1].respond(error=RuntimeError("connection lost"))
    harness.node.tick()
    assert not harness.node.state.connected
    assert harness.node.state.status == original
    assert "connection lost" in harness.node.state.connection_message
    harness.clock.advance(0.5)
    harness.node.tick()
    harness.reply_status()
    harness.switch.call_error = RuntimeError("cannot send")
    assert not harness.node.switch_controller("twist")
    assert not harness.node.state.operation_pending
    assert "cannot send" in harness.node.state.operation_message
    harness.switch.call_error = None
    harness.clock.advance(0.5)
    harness.node.tick()
    harness.reply_status()
    assert harness.node.switch_controller("twist")
    harness.switch.futures[-1].respond(error=RuntimeError("response lost"))
    harness.node.tick()
    assert not harness.node.state.operation_pending
    assert harness.node.state.operation_success is None
    assert "unknown" in harness.node.state.operation_message
    assert "response lost" in harness.node.state.operation_message
    assert not harness.node.state.connected
    harness.reply_status(status_message(active="twist"))
    assert harness.node.state.connected


def test_status_submit_failure_waits_for_next_poll_and_recovers(make_node):
    """Submission errors neither escape tick nor create a busy retry loop."""
    harness = make_node()
    harness.status.call_error = RuntimeError("cannot query")
    harness.node.tick()
    assert not harness.node.state.connected
    assert "cannot query" in harness.node.state.connection_message
    assert not harness.status.requests
    harness.status.call_error = None
    harness.node.tick()
    assert not harness.status.requests
    harness.clock.advance(0.5)
    harness.node.tick()
    harness.reply_status()
    assert harness.node.state.connected


@pytest.mark.parametrize("operation_pending", [False, True])
def test_close_retires_pending_work_and_ignores_late_completion(
    make_node, operation_pending
):
    """Closing is idempotent and cannot be undone by queued transport callbacks."""
    harness = make_node()
    if operation_pending:
        harness.connect()
        assert harness.node.switch_controller("twist")
        peer = harness.switch
    else:
        harness.node.tick()
        peer = harness.status
    pending = peer.futures[-1]
    harness.node.close()
    harness.node.close()
    assert pending.cancelled and pending in peer.retired
    assert not harness.node.state.connected
    assert not harness.node.state.operation_pending
    assert not harness.node.switch_controller("twist")
    assert not harness.node.reset_interrupt()
    pending.respond(peer.service.Response())
    harness.clock.advance(10.0)
    harness.node.tick()
    assert pending.result_reads == 0
    assert len(peer.requests) == 1
    assert not harness.node.state.connected
