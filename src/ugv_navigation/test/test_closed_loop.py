"""
Closed-loop A->B tests of the Dev 4 stack (dev.md Dev 4 tasks 2, 3, 6).

For each TEST-ONLY scenario (test/closed_loop/scenarios.py) this launches
closed_loop.launch.py: the real navigation.launch.py (Smac2D + RPP + BT + recoveries)
driving a kinematic fake base on /cmd_vel_nav2, with the scenario map fed to both
costmaps. Every scenario runs for both of Dev 5's robot footprints (primary and
secondary, applied through navigation.launch.py footprint_file). It sends a
NavigateToPose goal and checks that the robot arrives without its footprint polygon
touching lethal or unknown cells, respects the RPP
velocity bounds, reacts to a hazard that appears mid-run, and that nothing publishes
/cmd_vel. Per-scenario timings go to a benchmark CSV (UGV_CLOSED_LOOP_CSV).

It does not test perception, localization, Dev 3's real costmaps or Dev 5's safety
authority; those are stand-ins here.
"""

import csv
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml

PKG = Path(__file__).resolve().parent.parent
HARNESS = PKG / 'test' / 'closed_loop'
sys.path.insert(0, str(HARNESS))
sys.path.insert(0, str(PKG))

from scenarios import (  # noqa: E402, I100 (needs the sys.path entry above)
    cells_with, footprint_at, LETHAL, polygon_clearance, RESOLUTION, SCENARIOS, UNKNOWN)
from ugv_navigation.testbench_core import (  # noqa: E402
    format_summary, GoalRun, TwistStats, yaw_to_quaternion)

RPP = yaml.safe_load((PKG / 'config' / 'controller_server.yaml').read_text())[
    'controller_server']['ros__parameters']['FollowPath']
FIXTURES = PKG / 'test' / 'fixtures'
# Dev 5's footprints (TEST-ONLY copies until their PR lands).
ROBOTS = {name: FIXTURES / f'test_only_footprint_{name}.yaml'
          for name in ('primary', 'secondary')}
GOAL_TOLERANCE = 0.3            # goal checker xy 0.25 + integration slack
# The physical footprint (no padding) must stay off lethal AND unknown cells: its
# distance to every such cell centre must exceed half a cell.
MIN_CLEARANCE = RESOLUTION / 2
VEL_EPS = 1e-3
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
RESULTS = []


@pytest.fixture(scope='module', autouse=True)
def benchmark_report():
    yield
    if RESULTS:
        print('\n' + format_summary([r['run'] for r in RESULTS]))
        out = os.environ.get('UGV_CLOSED_LOOP_CSV')
        if out:
            with open(out, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['robot', 'scenario', 'status', 'elapsed_s', 'recoveries',
                                 'path_length_m', 'min_lethal_clearance_m',
                                 'min_unknown_clearance_m', 'cmd_msgs',
                                 'max_abs_v', 'max_abs_w'])
                for r in RESULTS:
                    run = r['run']
                    writer.writerow([r['robot'], r['scenario'], run.status, f'{run.elapsed_s:.2f}',
                                     run.recoveries, f'{r["length"]:.2f}',
                                     f'{r["clearance"]:.3f}', f'{r["unknown"]:.3f}',
                                     run.twist.count,
                                     f'{run.twist.max_abs_linear:.3f}',
                                     f'{run.twist.max_abs_angular:.3f}'])
            print(f'benchmark CSV: {out}')


class Recorder:
    """Records odometry poses, candidate twists and scenario map versions."""

    def __init__(self, node):
        self.poses = []
        self.stats = TwistStats()
        self.twists = []
        self.maps = []
        self.global_costmap = False
        node.create_subscription(Odometry, '/odom', self.on_odom, 50)
        node.create_subscription(Twist, '/cmd_vel_nav2', self.on_twist, 50)
        node.create_subscription(OccupancyGrid, '/test/scenario_map', self.maps.append, LATCHED)
        node.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self.on_costmap, LATCHED)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.poses.append((p.x, p.y, 2.0 * math.atan2(q.z, q.w)))

    def on_twist(self, msg):
        self.stats.add(msg.linear.x, msg.angular.z)
        self.twists.append((msg.linear.x, msg.angular.z))

    def on_costmap(self, _msg):
        self.global_costmap = True


def spin_until(executor, predicate, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        executor.spin_once(timeout_sec=0.05)
    return predicate()


def bt_navigator_active(node, executor):
    client = node.create_client(GetState, '/bt_navigator/get_state')
    try:
        if not client.wait_for_service(timeout_sec=0.5):
            return False
        future = client.call_async(GetState.Request())
        executor.spin_until_future_complete(future, timeout_sec=2.0)
        return future.done() and future.result().current_state.label == 'active'
    finally:
        node.destroy_client(client)


# Each scenario runs on its own ROS domain: processes of the previous scenario (the
# composed Nav2 container takes ~5 s to exit and its discovery entries linger ~15 s)
# can never be seen by the next one, and nothing has to wait for them to disappear.
_DOMAIN_COUNTER = iter(range(1, 1000))


def next_domain_id():
    base = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    return (base + next(_DOMAIN_COUNTER)) % 101  # 0..100: valid, no ephemeral ports


def run_scenario(scenario, footprint_file):
    domain_id = next_domain_id()
    proc = subprocess.Popen(
        ['ros2', 'launch', str(HARNESS / 'closed_loop.launch.py'),
         f'scenario:={scenario.name}', f'footprint_file:={footprint_file}'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True,
        env={**os.environ, 'ROS_DOMAIN_ID': str(domain_id)})
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=domain_id)
    executor = SingleThreadedExecutor(context=context)
    node = rclpy.create_node(f'closed_loop_test_{scenario.name}', context=context)
    executor.add_node(node)
    rec = Recorder(node)
    try:
        assert spin_until(executor, lambda: bt_navigator_active(node, executor), 60.0), \
            f'{scenario.name}: Nav2 did not become active'
        assert spin_until(executor, lambda: rec.global_costmap and rec.maps, 20.0), \
            f'{scenario.name}: scenario map never reached the global costmap'

        client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        assert client.wait_for_server(timeout_sec=10.0)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        x, y, yaw = scenario.goal
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        (goal.pose.pose.orientation.x, goal.pose.pose.orientation.y,
         goal.pose.pose.orientation.z, goal.pose.pose.orientation.w) = yaw_to_quaternion(yaw)

        recoveries = [0]

        def on_feedback(fb):
            recoveries[0] = fb.feedback.number_of_recoveries

        rec.poses.clear()
        start = time.monotonic()
        send = client.send_goal_async(goal, feedback_callback=on_feedback)
        executor.spin_until_future_complete(send, timeout_sec=10.0)
        handle = send.result()
        assert handle is not None and handle.accepted, f'{scenario.name}: goal rejected'
        result = handle.get_result_async()
        spin_until(executor, result.done, scenario.timeout_s)
        elapsed = time.monotonic() - start
        if not result.done():
            handle.cancel_goal_async()
            status, code, msg = 'TIMEOUT', -1, ''
        else:
            wrapped = result.result()
            status = 'SUCCEEDED' if wrapped.status == 4 else f'STATUS_{wrapped.status}'
            code, msg = wrapped.result.error_code, wrapped.result.error_msg
        spin_until(executor, lambda: False, 0.5)  # let the last odometry arrive
        final_cmd_vel = node.get_publishers_info_by_topic('/cmd_vel')
        run = GoalRun(scenario.goal, status, elapsed, recoveries[0], code, msg, rec.stats)
        return run, rec, final_cmd_vel
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
        os.killpg(proc.pid, signal.SIGINT)
        try:
            output, _ = proc.communicate(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            output, _ = proc.communicate()
        if os.environ.get('UGV_CLOSED_LOOP_VERBOSE'):
            print(output)


def clearance(poses, points, cells):
    """Min distance from the footprint polygon to `cells` over the run (10 Hz samples)."""
    return min((polygon_clearance(footprint_at(points, x, y, yaw), cells)
                for x, y, yaw in poses[::5]), default=float('inf'))


@pytest.mark.parametrize('robot', list(ROBOTS))
@pytest.mark.parametrize('name', list(SCENARIOS))
def test_closed_loop_scenario(name, robot):
    scenario = SCENARIOS[name]
    run, rec, cmd_vel_publishers = run_scenario(scenario, ROBOTS[robot])
    points = yaml.safe_load(yaml.safe_load(ROBOTS[robot].read_text())['footprint'])
    final_map = rec.maps[-1].data
    lethal = clearance(rec.poses, points, cells_with(final_map, LETHAL))
    unknown = clearance(rec.poses, points, cells_with(final_map, UNKNOWN))
    xy = [(x, y) for x, y, _ in rec.poses]
    length = sum(math.dist(a, b) for a, b in zip(xy, xy[1:]))
    RESULTS.append({'robot': robot, 'scenario': name, 'run': run, 'length': length,
                    'clearance': lethal, 'unknown': unknown})

    tag = f'{robot}/{name}'
    assert run.status == 'SUCCEEDED', f'{tag}: {run.status} {run.error_code} {run.error_msg}'
    gx, gy, _ = scenario.goal
    fx, fy = xy[-1]
    assert math.hypot(fx - gx, fy - gy) <= GOAL_TOLERANCE, f'{tag}: stopped at {fx, fy}'

    # Footprint polygon never overlaps a lethal cell ...
    assert lethal >= MIN_CLEARANCE, f'{tag}: footprint within {lethal:.3f} m of lethal'
    # ... nor an unknown one (unknown != free, arch §8.1).
    assert unknown >= MIN_CLEARANCE, f'{tag}: footprint within {unknown:.3f} m of unknown'

    # Candidate twists stay inside the RPP velocity window (BackUp may reverse).
    assert rec.stats.max_abs_linear <= RPP['max_linear_vel'] + VEL_EPS
    assert rec.stats.max_abs_angular <= RPP['max_angular_vel'] + VEL_EPS
    if run.recoveries == 0:
        assert rec.stats.min_linear >= -VEL_EPS, f'{tag}: reversed without a recovery'
    # Boundary: only Dev 5 may publish /cmd_vel; nothing here does.
    assert cmd_vel_publishers == []

    if name == 'wall_gap':
        crossing = [y for x, y in xy if abs(x - 2.0) < 0.1]
        assert crossing and all(1.0 < y < 1.8 for y in crossing), 'did not use the opening'
    if name == 'corridor':
        inside = [y for x, y in xy if 1.0 < x < 3.0]
        assert inside and all(abs(y) < 0.6 for y in inside), 'left the corridor'
    if name == 'dynamic_obstacle':
        assert len(rec.maps) >= 2, 'the late hazard was never published'
