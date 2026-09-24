"""
Live ROS graph check of navigation.launch.py (Dev 4 boundary).

Launches the real launch file with autostart:=false and a TEST-ONLY costmap fixture,
then CONFIGURES the Nav2 servers. Configuring loads the Smac2D, RPP, behavior and
navigator plugins with the Dev 4 params and creates every publisher and action server,
without needing TF, odometry or costmap data from Dev 2/3. bt_navigator alone is then
ACTIVATED, which makes BehaviorTree.CPP load and build navigate_to_pose_ugv.xml against
the live planner / controller / behavior action servers. The planner and controller
are never activated (that needs Dev 2 TF).

It does not drive the robot and does not claim end-to-end navigation works.
"""

import os
from pathlib import Path
import signal
import subprocess
import time

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import NavigateToPose
import pytest
import rclpy
from rclpy.action import ActionClient

PKG = Path(__file__).resolve().parent.parent
LAUNCH_FILE = PKG / 'launch' / 'navigation.launch.py'
FIXTURE = PKG / 'test' / 'fixtures' / 'test_only_costmaps.yaml'
SERVERS = ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
TWIST = 'geometry_msgs/msg/Twist'


def change_state(node, name, transition_id):
    client = node.create_client(ChangeState, f'/{name}/change_state')
    assert client.wait_for_service(timeout_sec=30.0), f'{name} did not start'
    request = ChangeState.Request()
    request.transition.id = transition_id
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    node.destroy_client(client)
    return future.done() and future.result().success


@pytest.fixture(scope='module')
def configured_stack():
    proc = subprocess.Popen(
        ['ros2', 'launch', str(LAUNCH_FILE), f'costmap_params_file:={FIXTURE}',
         'autostart:=false'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('dev4_graph_test')
    try:
        for name in SERVERS:
            assert change_state(node, name, Transition.TRANSITION_CONFIGURE), \
                f'{name} failed to configure'
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(proc.pid, signal.SIGINT)
        try:
            output, _ = proc.communicate(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            output, _ = proc.communicate()
        print(output)


def wait_for_publishers(node, topic, expected, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        infos = node.get_publishers_info_by_topic(topic)
        if {i.node_name for i in infos} >= expected:
            return infos
        rclpy.spin_once(node, timeout_sec=0.1)
    return node.get_publishers_info_by_topic(topic)


def test_candidate_topic_published_by_controller_and_behaviors(configured_stack):
    infos = wait_for_publishers(configured_stack, '/cmd_vel_nav2',
                                {'controller_server', 'behavior_server'})
    assert {i.node_name for i in infos} == {'controller_server', 'behavior_server'}
    assert {i.topic_type for i in infos} == {TWIST}


def test_nothing_publishes_final_cmd_vel(configured_stack):
    rclpy.spin_once(configured_stack, timeout_sec=0.5)
    assert configured_stack.get_publishers_info_by_topic('/cmd_vel') == []


def test_navigate_to_pose_action_server_exists(configured_stack):
    client = ActionClient(configured_stack, NavigateToPose, '/navigate_to_pose')
    assert client.wait_for_server(timeout_sec=10.0)
    client.destroy()


def test_bt_navigator_activates_with_ugv_behavior_tree(configured_stack):
    # Runs last: activation parses the BT XML and instantiates every BT node.
    assert change_state(configured_stack, 'bt_navigator', Transition.TRANSITION_ACTIVATE)
