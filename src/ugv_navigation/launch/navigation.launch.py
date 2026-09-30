"""
Dev 4 Nav2 planning & control stack: Smac2D + RPP + BT Navigator + recoveries.

Launches only Dev 4 nodes:
    planner_server (Smac2D, hosts global_costmap)
    controller_server (RPP, hosts local_costmap)
    behavior_server (spin / wait / backup)
    bt_navigator (/navigate_to_pose)
    lifecycle_manager_navigation
    nav2_heartbeat (/ugv/nav2_heartbeat for Dev 5's watchdog, architecture §12)

use_composition:=true (default) runs the first five as components in one
nav2_container process (as Nav2's own bringup does): less memory and CPU than five
processes. nav2_heartbeat always runs in its own process so it keeps publishing
`false` if the container dies.

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

import hashlib
from pathlib import Path
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import yaml

# Resolves to src/ugv_navigation (source) or share/ugv_navigation (install).
PKG_DIR = Path(__file__).resolve().parent.parent
CONFIG_DIR = PKG_DIR / 'config'
DEFAULT_BT_XML = PKG_DIR / 'behavior_trees' / 'navigate_to_pose_ugv.xml'
# Owned and written by Dev 3 (costmap subsystem). Dev 4 only looks it up.
DEFAULT_COSTMAP_PARAMS = CONFIG_DIR / 'costmaps.yaml'
# Dev 4 per-robot speed / acceleration overlays: config/robots/<robot>/nav2_limits.yaml
ROBOTS_DIR = CONFIG_DIR / 'robots'

CANDIDATE_CMD_VEL_TOPIC = '/cmd_vel_nav2'
# Nav2 servers publish on relative `cmd_vel`; route them to the candidate topic.
CMD_VEL_REMAP = [('cmd_vel', CANDIDATE_CMD_VEL_TOPIC)]

# (package, executable, component plugin, node name, Dev 4 params file, publishes cmd_vel)
NAV2_SERVERS = [
    ('nav2_planner', 'planner_server', 'nav2_planner::PlannerServer', 'planner_server',
     'planner_server.yaml', False),
    ('nav2_controller', 'controller_server', 'nav2_controller::ControllerServer',
     'controller_server', 'controller_server.yaml', True),
    ('nav2_behaviors', 'behavior_server', 'behavior_server::BehaviorServer',
     'behavior_server', 'behavior_server.yaml', True),
    ('nav2_bt_navigator', 'bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator',
     'bt_navigator.yaml', False),
]
LIFECYCLE_NODES = [server[3] for server in NAV2_SERVERS]
CONTAINER_NAME = 'nav2_container'
# Nodes hosting a costmap (they get the robot footprint).
COSTMAP_HOSTS = {'planner_server': 'global_costmap', 'controller_server': 'local_costmap'}


def resolve_robot_params(robot, robot_params_file):
    """robot:=<name> -> config/robots/<name>/nav2_limits.yaml, or the explicit file."""
    if robot and robot_params_file:
        raise RuntimeError('navigation.launch.py: pass robot:=<name> or robot_params_file, '
                           'not both.')
    if not robot:
        return robot_params_file
    path = ROBOTS_DIR / robot / 'nav2_limits.yaml'
    if not path.is_file():
        known = sorted(d.name for d in ROBOTS_DIR.iterdir() if d.is_dir())
        raise RuntimeError(f'navigation.launch.py: unknown robot "{robot}" '
                           f'(known: {", ".join(known)})')
    return str(path)


def find_footprint_file(robot, start=PKG_DIR):
    """
    Dev 5's config/robots/footprint_<robot>.yaml, searched upward from this package.

    Found from the source tree and from an install space built inside the repo. The
    search stops at the repository root (the first directory holding `.git`) so an
    unrelated file higher up is never picked. Returns '' if not found (the footprint
    then comes from Dev 3's costmap params).
    """
    for parent in [start, *start.parents]:
        candidate = parent / 'config' / 'robots' / f'footprint_{robot}.yaml'
        if candidate.is_file():
            return str(candidate)
        if (parent / '.git').exists():
            break
    return ''


def load_footprint(path):
    """Read Dev 5's footprint format: `footprint` (polygon string) + `footprint_padding`."""
    data = yaml.safe_load(Path(path).read_text()) or {}
    footprint = data.get('footprint')
    points = yaml.safe_load(footprint) if isinstance(footprint, str) else None
    if not (isinstance(points, list) and len(points) >= 3 and
            all(isinstance(pt, list) and len(pt) == 2 for pt in points)):
        raise RuntimeError(f'navigation.launch.py: {path}: `footprint` must be a polygon '
                           'string like "[[x, y], [x, y], [x, y]]"')
    return {'footprint': footprint, 'footprint_padding': float(data.get('footprint_padding', 0.0))}


def footprint_params_file(footprint):
    """
    Write the footprint as costmap parameters for both costmaps; return the path.

    Content-addressed: the same footprint always maps to the same file, so repeated
    launches reuse it instead of leaving a new temp file behind each time.
    """
    params = {costmap: {costmap: {'ros__parameters': dict(footprint)}}
              for costmap in COSTMAP_HOSTS.values()}
    text = yaml.safe_dump(params, sort_keys=True)
    digest = hashlib.sha256(text.encode()).hexdigest()[:16]
    path = Path(tempfile.gettempdir()) / f'ugv_footprint_{digest}.yaml'
    if not path.is_file() or path.read_text() != text:
        tmp = path.with_suffix(f'.{digest}.tmp')
        tmp.write_text(text)
        tmp.replace(path)  # atomic: concurrent launches never read a partial file
    return str(path)


def parameter_files(params_file, costmap_params_file, robot_params_file,
                    footprint_file=''):
    """Later entries override: Dev 4 defaults < Dev 3 costmaps < footprint < robot."""
    files = [str(CONFIG_DIR / params_file), costmap_params_file]
    if footprint_file:
        files.append(footprint_file)
    if robot_params_file:
        files.append(robot_params_file)
    return files


def nav2_specs(costmap_params_file, robot_params_file, footprint_params, bt_xml, common,
               autostart):
    """
    Plain description of every Dev 4 node (no launch objects, unit-testable).

    Each spec: name, package, executable, plugin (None = never composed), parameters
    (later entries override earlier ones), remappings.
    """
    specs = []
    for package, executable, plugin, name, params_file, publishes_cmd_vel in NAV2_SERVERS:
        # Files (Dev 4 defaults < Dev 3 costmaps < footprint < robot) < launch overrides.
        parameters = parameter_files(
            params_file, costmap_params_file, robot_params_file,
            footprint_params if name in COSTMAP_HOSTS else '')
        parameters.append(common)
        if name == 'bt_navigator':
            parameters.append({'default_nav_to_pose_bt_xml': bt_xml})
        specs.append({'name': name, 'package': package, 'executable': executable,
                      'plugin': plugin, 'parameters': parameters,
                      'remappings': CMD_VEL_REMAP if publishes_cmd_vel else []})
    specs.append({'name': 'lifecycle_manager_navigation',
                  'package': 'nav2_lifecycle_manager', 'executable': 'lifecycle_manager',
                  'plugin': 'nav2_lifecycle_manager::LifecycleManager',
                  'parameters': [common, {'autostart': autostart,
                                          'node_names': LIFECYCLE_NODES}],
                  'remappings': []})
    # Never composed: it must keep publishing (false) while Nav2 or its container is down.
    specs.append({'name': 'nav2_heartbeat', 'package': 'ugv_navigation',
                  'executable': 'nav2_heartbeat', 'plugin': None,
                  'parameters': [common, {'servers': LIFECYCLE_NODES}], 'remappings': []})
    return specs


def container_parameter_files(composed):
    """
    Every parameter file of the composed nodes, for the container process itself.

    Nav2 servers create their costmaps (global_costmap, local_costmap) as child nodes
    that read their sections from the process's --params-file arguments, not from the
    component's own parameters. Without these files on the container the costmaps
    silently fall back to Nav2's default layers. Order kept: Dev 4 defaults < Dev 3
    costmaps < footprint < robot overlay.
    """
    files = []
    for spec in composed:
        for entry in spec['parameters']:
            if isinstance(entry, str) and entry not in files:
                files.append(entry)
    defaults = [f for f in files if Path(f).parent == CONFIG_DIR]
    return defaults + [f for f in files if f not in defaults]


def nav2_actions(specs, use_composition, use_sim_time, ros_args):
    """Composable specs go into one container when use_composition, the rest run alone."""
    composed = [s for s in specs if use_composition and s['plugin']]
    actions = [Node(package=s['package'], executable=s['executable'], name=s['name'],
                    output='screen', respawn=False, parameters=s['parameters'],
                    remappings=s['remappings'], arguments=ros_args)
               for s in specs if s not in composed]
    if composed:
        actions.insert(0, ComposableNodeContainer(
            name=CONTAINER_NAME, namespace='', package='rclcpp_components',
            executable='component_container_isolated', output='screen',
            arguments=ros_args,
            parameters=container_parameter_files(composed) + [{'use_sim_time': use_sim_time}],
            composable_node_descriptions=[
                ComposableNode(package=s['package'], plugin=s['plugin'], name=s['name'],
                               parameters=s['parameters'], remappings=s['remappings'])
                for s in composed]))
    return actions


def _launch_setup(context, *args, **kwargs):
    costmap_params_file = LaunchConfiguration('costmap_params_file').perform(context)
    robot = LaunchConfiguration('robot').perform(context)
    robot_params_file = LaunchConfiguration('robot_params_file').perform(context)
    footprint_file = LaunchConfiguration('footprint_file').perform(context)
    bt_xml = LaunchConfiguration('bt_xml').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    autostart = LaunchConfiguration('autostart').perform(context).lower() == 'true'
    log_level = LaunchConfiguration('log_level').perform(context)
    use_composition = \
        LaunchConfiguration('use_composition').perform(context).lower() == 'true'

    robot_params_file = resolve_robot_params(robot, robot_params_file)
    # Footprint selection (Dev 5 files, Dev 4 selects): explicit file, else Dev 5's
    # footprint_<robot>.yaml if reachable, else whatever Dev 3's costmap params set.
    if not footprint_file and robot:
        footprint_file = find_footprint_file(robot)
    actions = [LogInfo(msg='navigation.launch.py: footprint from ' + (
        footprint_file or "Dev 3's costmap params (no footprint_file / Dev 5 file found)"))]
    footprint_params = ''
    if footprint_file:
        if not Path(footprint_file).is_file():
            raise RuntimeError(f'navigation.launch.py: footprint_file not found: '
                               f'{footprint_file}')
        footprint_params = footprint_params_file(load_footprint(footprint_file))
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

    specs = nav2_specs(costmap_params_file, robot_params_file, footprint_params, bt_xml,
                       {'use_sim_time': use_sim_time}, autostart)
    return actions + nav2_actions(specs, use_composition, use_sim_time,
                                  ['--ros-args', '--log-level', log_level])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'costmap_params_file', default_value='',
            description='Dev 3 costmap params (global_costmap / local_costmap). '
                        'Default: config/costmaps.yaml (Dev 3) if present.'),
        DeclareLaunchArgument(
            'robot', default_value='',
            description='Robot overlay config/robots/<robot>/nav2_limits.yaml '
                        '(e.g. primary, secondary). Empty = Dev 4 defaults.'),
        DeclareLaunchArgument(
            'robot_params_file', default_value='',
            description='Optional per-robot overlay from config/robots/<robot>/ '
                        '(speed / acceleration limits for RPP and recoveries).'),
        DeclareLaunchArgument(
            'footprint_file', default_value='',
            description='Robot footprint in Dev 5 format (footprint + footprint_padding), '
                        'applied to both costmaps. Default with robot:=<name>: Dev 5 '
                        'config/robots/footprint_<name>.yaml if found above this package.'),
        DeclareLaunchArgument(
            'bt_xml', default_value=str(DEFAULT_BT_XML),
            description='NavigateToPose behavior tree XML.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Configure + activate Nav2 servers automatically. Activation '
                        'waits for TF map->odom->base_link (Dev 2).'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'use_composition', default_value='true',
            description='Run the Nav2 servers + lifecycle manager as components in one '
                        'container (true) or as separate processes (false, debugging).'),
        OpaqueFunction(function=_launch_setup),
    ])
