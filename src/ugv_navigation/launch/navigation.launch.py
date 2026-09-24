"""
Dev 4 Nav2 planning & control stack: Smac2D + RPP + BT Navigator + recoveries.

Launches only Dev 4 nodes:
    planner_server (Smac2D, hosts global_costmap)
    controller_server (RPP, hosts local_costmap)
    behavior_server (spin / wait / backup)
    bt_navigator (/navigate_to_pose)
    lifecycle_manager_navigation

Does NOT launch camera, perception, RTAB-Map / localization, TF, robot_state_publisher,
map_server, safety authority, velocity smoother, collision monitor or motor driver.
Those are external subsystems (Dev 1/2/3/5).

Output boundary (architecture.md §3.1): every Nav2 `cmd_vel` publisher is remapped to
/cmd_vel_nav2. Nothing launched here publishes /cmd_vel; Dev 5 owns it.

Costmaps: global_costmap / local_costmap run inside planner_server / controller_server,
but their layers, footprint and inflation are Dev 3's. They are read from
costmap_params_file:=<path>, or by default from Dev 3's config/costmaps.yaml in this
package when that file exists. With neither, the launch refuses to start rather than
silently running Nav2's default costmap layers.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Resolves to src/ugv_navigation (source) or share/ugv_navigation (install).
PKG_DIR = Path(__file__).resolve().parent.parent
CONFIG_DIR = PKG_DIR / 'config'
DEFAULT_BT_XML = PKG_DIR / 'behavior_trees' / 'navigate_to_pose_ugv.xml'
# Owned and written by Dev 3 (costmap subsystem). Dev 4 only looks it up.
DEFAULT_COSTMAP_PARAMS = CONFIG_DIR / 'costmaps.yaml'

CANDIDATE_CMD_VEL_TOPIC = '/cmd_vel_nav2'
# Nav2 servers publish on relative `cmd_vel`; route them to the candidate topic.
CMD_VEL_REMAP = [('cmd_vel', CANDIDATE_CMD_VEL_TOPIC)]

# (package, executable, node name, Dev 4 params file, publishes cmd_vel)
NAV2_SERVERS = [
    ('nav2_planner', 'planner_server', 'planner_server', 'planner_server.yaml', False),
    ('nav2_controller', 'controller_server', 'controller_server', 'controller_server.yaml',
     True),
    ('nav2_behaviors', 'behavior_server', 'behavior_server', 'behavior_server.yaml', True),
    ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', 'bt_navigator.yaml', False),
]
LIFECYCLE_NODES = [name for _, _, name, _, _ in NAV2_SERVERS]


def _launch_setup(context, *args, **kwargs):
    costmap_params_file = LaunchConfiguration('costmap_params_file').perform(context)
    robot_params_file = LaunchConfiguration('robot_params_file').perform(context)
    bt_xml = LaunchConfiguration('bt_xml').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    autostart = LaunchConfiguration('autostart').perform(context).lower() == 'true'
    log_level = LaunchConfiguration('log_level').perform(context)

    if not costmap_params_file and DEFAULT_COSTMAP_PARAMS.is_file():
        costmap_params_file = str(DEFAULT_COSTMAP_PARAMS)
    if not costmap_params_file:
        raise RuntimeError(
            'navigation.launch.py: no costmap params. The global/local costmap layers, '
            'footprint and inflation are owned by Dev 3 (ugv_navigation costmap subsystem '
            f'/ config/robots). Dev 3 provides {DEFAULT_COSTMAP_PARAMS}, or pass '
            'costmap_params_file:=<path to Dev 3 costmap yaml>.')
    for label, path in (('costmap_params_file', costmap_params_file),
                        ('robot_params_file', robot_params_file),
                        ('bt_xml', bt_xml)):
        if path and not Path(path).is_file():
            raise RuntimeError(f'navigation.launch.py: {label} not found: {path}')

    common = {'use_sim_time': use_sim_time}
    nodes = []
    for package, executable, name, params_file, publishes_cmd_vel in NAV2_SERVERS:
        # Later entries override earlier ones: Dev 4 defaults < Dev 3 costmaps <
        # per-robot overlay < launch overrides.
        parameters = [str(CONFIG_DIR / params_file), costmap_params_file]
        if robot_params_file:
            parameters.append(robot_params_file)
        parameters.append(common)
        if name == 'bt_navigator':
            parameters.append({'default_nav_to_pose_bt_xml': bt_xml})
        nodes.append(Node(
            package=package,
            executable=executable,
            name=name,
            output='screen',
            respawn=False,
            parameters=parameters,
            remappings=CMD_VEL_REMAP if publishes_cmd_vel else [],
            arguments=['--ros-args', '--log-level', log_level],
        ))

    nodes.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[common, {'autostart': autostart, 'node_names': LIFECYCLE_NODES}],
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'costmap_params_file', default_value='',
            description='Dev 3 costmap params (global_costmap / local_costmap). '
                        'Default: config/costmaps.yaml (Dev 3) if present.'),
        DeclareLaunchArgument(
            'robot_params_file', default_value='',
            description='Optional per-robot overlay from config/robots/<robot>/ '
                        '(speed / acceleration limits for RPP and recoveries).'),
        DeclareLaunchArgument(
            'bt_xml', default_value=str(DEFAULT_BT_XML),
            description='NavigateToPose behavior tree XML.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Configure + activate Nav2 servers automatically. Activation '
                        'waits for TF map->odom->base_link (Dev 2).'),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_launch_setup),
    ])
