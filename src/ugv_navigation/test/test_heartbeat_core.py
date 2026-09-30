"""Unit tests for the ROS-free Nav2 heartbeat logic."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ugv_navigation.heartbeat_core import DEFAULT_SERVERS, HeartbeatMonitor  # noqa: E402


def all_active(monitor, now):
    for server in monitor.servers:
        monitor.record(server, 'active', now)


def test_fails_closed_before_any_reply():
    healthy, reason = HeartbeatMonitor().evaluate(0.0)
    assert not healthy and 'not active' in reason


def test_healthy_when_every_server_recently_active():
    monitor = HeartbeatMonitor()
    all_active(monitor, 10.0)
    assert monitor.evaluate(10.4) == (True, 'ok')


def test_one_inactive_server_fails():
    monitor = HeartbeatMonitor()
    all_active(monitor, 10.0)
    monitor.record(DEFAULT_SERVERS[-1], 'inactive', 10.05)
    healthy, reason = monitor.evaluate(10.1)
    assert not healthy and DEFAULT_SERVERS[-1] in reason and 'inactive' in reason


def test_stale_reply_fails():
    # A crashed or hung server stops answering: its last 'active' ages out.
    monitor = HeartbeatMonitor(max_age_s=0.5)
    all_active(monitor, 10.0)
    for server in DEFAULT_SERVERS[1:]:
        monitor.record(server, 'active', 10.6)
    healthy, reason = monitor.evaluate(10.6)
    assert not healthy and DEFAULT_SERVERS[0] in reason and 'stale' in reason


def test_deactivation_fails_immediately():
    # A fresh 'active' from before must not mask a newer 'inactive' reply.
    monitor = HeartbeatMonitor(max_age_s=0.5)
    all_active(monitor, 10.0)
    monitor.record('controller_server', 'inactive', 10.2)
    healthy, reason = monitor.evaluate(10.21)
    assert not healthy and 'controller_server' in reason and 'inactive' in reason


def test_recovers_when_server_is_active_again():
    monitor = HeartbeatMonitor(max_age_s=0.5)
    all_active(monitor, 10.0)
    assert not monitor.evaluate(11.0)[0]
    all_active(monitor, 11.0)
    assert monitor.evaluate(11.1) == (True, 'ok')


def test_age_check_can_be_skipped_after_a_monitor_stall():
    monitor = HeartbeatMonitor(max_age_s=0.5)
    all_active(monitor, 10.0)
    # The monitoring process was paused: old reply times are not evidence of a fault ...
    assert monitor.evaluate(11.0, check_age=False) == (True, 'ok')
    # ... but an explicit non-active reply still fails immediately.
    monitor.record('planner_server', 'inactive', 11.0)
    assert not monitor.evaluate(11.0, check_age=False)[0]
