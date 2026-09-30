"""
TEST-ONLY closed-loop harness for the Dev 4 stack.

Runs the real navigation.launch.py with test-only stand-ins for the other devs:
    Dev 3 costmap   -> scenario_map_node (synthetic map via StaticLayer fixture)
    Dev 2 TF / odom -> identity map->odom + fake_base_node odom->base_link, /odom
    Dev 5 base      -> fake_base_node integrates /cmd_vel_nav2 (no /cmd_vel)

    ros2 launch src/ugv_navigation/test/closed_loop/closed_loop.launch.py scenario:=wall_gap \
        footprint_file:=src/ugv_navigation/test/fixtures/test_only_footprint_primary.yaml
    ros2 run ugv_navigation nav_goal_testbench goal 4.0,0.0
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent
PKG = HERE.parent.parent
COSTMAPS = PKG / 'test' / 'fixtures' / 'test_only_closed_loop_costmaps.yaml'


def generate_launch_description():
    scenario = LaunchConfiguration('scenario')
    return LaunchDescription([
        DeclareLaunchArgument('footprint_file', default_value='',
                              description='Robot footprint (Dev 5 format); empty = the '
                                          'fixture robot_radius 0.2 m'),
        DeclareLaunchArgument('use_composition', default_value='true'),
        DeclareLaunchArgument('scenario', default_value='open',
                              description='open | wall_gap | corridor | unknown_block | '
                                          'dynamic_obstacle'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='test_map_to_odom', output='log',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom']),
        ExecuteProcess(cmd=['python3', str(HERE / 'fake_base_node.py')], output='screen'),
        ExecuteProcess(
            cmd=['python3', str(HERE / 'scenario_map_node.py'),
                 '--ros-args', '-p', ['scenario:=', scenario]],
            output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(PKG / 'launch' / 'navigation.launch.py')),
            launch_arguments={'costmap_params_file': str(COSTMAPS),
                              'footprint_file': LaunchConfiguration('footprint_file'),
                              'use_composition': LaunchConfiguration('use_composition')}.items()),
    ])
