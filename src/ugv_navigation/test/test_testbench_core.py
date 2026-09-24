"""Unit tests for the ROS-free testbench logic."""

import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ugv_navigation.testbench_core import (  # noqa: E402
    Endpoint, evaluate_boundary, format_summary, GoalRun, parse_goal, TwistStats,
    yaw_to_quaternion)

TWIST = 'geometry_msgs/msg/Twist'


def test_boundary_passes_with_dev4_candidates_and_external_final():
    report = evaluate_boundary(
        [Endpoint('controller_server', '/', TWIST), Endpoint('behavior_server', '/', TWIST)],
        [Endpoint('safety_arbiter', '/', TWIST)])
    assert report.ok
    assert not report.warnings


def test_boundary_reports_each_node_once():
    report = evaluate_boundary([Endpoint('behavior_server', '/', TWIST)] * 3, [])
    assert len(report.info) == 1


def test_boundary_fails_if_dev4_publishes_final_cmd_vel():
    report = evaluate_boundary([Endpoint('controller_server', '/', TWIST)],
                               [Endpoint('controller_server', '/', TWIST)])
    assert not report.ok


def test_boundary_fails_on_stamped_candidate():
    report = evaluate_boundary(
        [Endpoint('controller_server', '/', 'geometry_msgs/msg/TwistStamped')], [])
    assert not report.ok


def test_boundary_warns_when_nothing_publishes_candidate():
    report = evaluate_boundary([], [])
    assert report.ok and report.warnings


def test_parse_goal():
    assert parse_goal('3,0') == (3.0, 0.0, 0.0)
    assert parse_goal(' 1.5, -2 , 1.57 ') == (1.5, -2.0, 1.57)
    for bad in ('3', '1,2,3,4', '1,,2', 'a,b', 'nan,0'):
        with pytest.raises(ValueError):
            parse_goal(bad)


def test_yaw_to_quaternion():
    assert yaw_to_quaternion(0.0) == pytest.approx((0.0, 0.0, 0.0, 1.0))
    _, _, z, w = yaw_to_quaternion(math.pi / 2)
    assert (z, w) == pytest.approx((math.sqrt(0.5), math.sqrt(0.5)))


def test_twist_stats_and_summary():
    stats = TwistStats()
    for v, w in ((0.0, 0.0), (0.3, -0.5), (-0.1, 0.2)):
        stats.add(v, w)
    assert (stats.count, stats.moving) == (3, 2)
    assert stats.max_abs_linear == pytest.approx(0.3)
    assert stats.max_abs_angular == pytest.approx(0.5)
    assert stats.min_linear == pytest.approx(-0.1)
    table = format_summary([GoalRun((3.0, 0.0, 0.0), 'SUCCEEDED', 12.5, 1, 0, '', stats)])
    assert 'SUCCEEDED' in table and '12.50' in table
