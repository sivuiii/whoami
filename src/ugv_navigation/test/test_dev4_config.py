"""
Static checks of the Dev 4 Nav2 configuration against the project contract.

No ROS graph is needed. Plugin names and parameter names are checked against the
installed Nav2 so typos (which Nav2 silently ignores) fail here.
"""

import importlib.util
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchContext
import pytest
import yaml

PKG = Path(__file__).resolve().parent.parent
CONFIG = PKG / 'config'
BT_XML = PKG / 'behavior_trees' / 'navigate_to_pose_ugv.xml'
LAUNCH_FILE = PKG / 'launch' / 'navigation.launch.py'
FIXTURE_COSTMAPS = PKG / 'test' / 'fixtures' / 'test_only_costmaps.yaml'
DEV4_YAMLS = ['planner_server.yaml', 'controller_server.yaml', 'behavior_server.yaml',
              'bt_navigator.yaml']

# BehaviorTree.CPP built-in control / decorator nodes allowed in the tree.
BT_BUILTINS = {'Sequence', 'SequenceWithMemory', 'ReactiveSequence', 'Fallback',
               'ReactiveFallback', 'Parallel', 'Inverter', 'ForceSuccess', 'ForceFailure',
               'Repeat', 'RetryUntilSuccessful', 'KeepRunningUntilFailure', 'SubTree'}


def params(file_name, node):
    with open(CONFIG / file_name) as f:
        return yaml.safe_load(f)[node]['ros__parameters']


def plugin_registered(package, class_type):
    share = Path(get_package_share_directory(package))
    return any(f'type="{class_type}"' in p.read_text() for p in share.glob('*.xml'))


def load_launch_module():
    spec = importlib.util.spec_from_file_location('navigation_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# ---------------------------------------------------------------- 1. Smac2D planner

def test_planner_is_smac2d_and_installed():
    p = params('planner_server.yaml', 'planner_server')
    assert p['planner_plugins'] == ['GridBased']
    assert p['GridBased']['plugin'] == 'nav2_smac_planner::SmacPlanner2D'
    assert plugin_registered('nav2_smac_planner', 'nav2_smac_planner::SmacPlanner2D')


def test_planner_never_treats_unknown_as_free():
    assert params('planner_server.yaml', 'planner_server')['GridBased']['allow_unknown'] is False


def test_planner_bounded_for_continuous_replanning():
    g = params('planner_server.yaml', 'planner_server')['GridBased']
    assert 0 < g['max_planning_time'] <= 2.0
    assert g['cost_travel_multiplier'] > 0


# ---------------------------------------------------------------- 2. RPP controller

def test_controller_is_rpp_and_installed():
    p = params('controller_server.yaml', 'controller_server')
    cls = 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
    assert p['controller_plugins'] == ['FollowPath']
    assert p['FollowPath']['plugin'] == cls
    assert plugin_registered('nav2_regulated_pure_pursuit_controller', cls)


def test_rpp_parameter_names_exist_in_installed_nav2():
    pkg = 'nav2_regulated_pure_pursuit_controller'
    # Lyrical installs headers under include/<pkg>/<pkg>/, older distros under include/<pkg>/.
    headers = sorted((Path(get_package_prefix(pkg)) / 'include').glob(
        f'**/{pkg}/parameter_handler.hpp'))
    assert headers, f'{pkg} parameter_handler.hpp not installed'
    header = headers[0]
    known = set()
    for decl in re.findall(r'^\s+(?:double|bool|int)\s+([^;(]+);', header.read_text(), re.M):
        known.update(name.strip() for name in decl.split(','))
    rpp = params('controller_server.yaml', 'controller_server')['FollowPath']
    unknown = set(rpp) - known - {'plugin'}
    assert not unknown, f'RPP params not in installed Nav2: {sorted(unknown)}'


def test_rpp_adaptive_lookahead():
    r = params('controller_server.yaml', 'controller_server')['FollowPath']
    assert r['use_velocity_scaled_lookahead_dist'] is True
    assert 0 < r['min_lookahead_dist'] <= r['lookahead_dist'] <= r['max_lookahead_dist']
    assert r['lookahead_time'] > 0


def test_rpp_curvature_and_collision_aware_regulation():
    r = params('controller_server.yaml', 'controller_server')['FollowPath']
    assert r['use_regulated_linear_velocity_scaling'] is True
    assert r['use_cost_regulated_linear_velocity_scaling'] is True
    assert r['use_collision_detection'] is True
    assert r['max_allowed_time_to_collision_up_to_carrot'] > 0
    assert 0 < r['regulated_linear_scaling_min_speed'] < r['max_linear_vel']


def test_rpp_differential_drive():
    r = params('controller_server.yaml', 'controller_server')['FollowPath']
    assert r['use_rotate_to_heading'] is True
    # RPP rejects rotate-to-heading together with reversing.
    assert r['allow_reversing'] is False


def test_rpp_velocity_and_accel_bounds_are_enforced():
    r = params('controller_server.yaml', 'controller_server')['FollowPath']
    assert 'desired_linear_vel' not in r  # deprecated name in Lyrical
    # RPP only applies min/max velocity and decel limits inside the dynamic window.
    assert r['use_dynamic_window'] is True
    # Lyrical defaults are min_linear_vel -0.5, |angular| 2.5 rad/s, accel 2.5 m/s^2.
    assert r['min_linear_vel'] == 0.0 < r['max_linear_vel']
    assert r['min_angular_vel'] == -r['max_angular_vel'] < 0
    assert r['max_linear_accel'] > 0 > r['max_linear_decel']
    assert r['max_angular_accel'] > 0 > r['max_angular_decel']
    # Rotate-to-heading runs outside the window.
    assert r['rotate_to_heading_angular_vel'] <= r['max_angular_vel']


def test_bt_follow_path_names_its_plugins():
    p = params('controller_server.yaml', 'controller_server')
    follow = bt_root().find('.//FollowPath')
    assert follow.get('goal_checker_id') in p['goal_checker_plugins']
    assert follow.get('progress_checker_id') in p['progress_checker_plugins']
    assert follow.get('path_handler_id') in p['path_handler_plugins']


def test_controller_path_handler_is_installed():
    p = params('controller_server.yaml', 'controller_server')
    assert p['path_handler_plugins'] == ['PathHandler']
    cls = 'nav2_controller::FeasiblePathHandler'
    assert p['PathHandler']['plugin'] == cls
    assert plugin_registered('nav2_controller', cls)


# ---------------------------------------------------------------- 3. Dynamic reactivity

def test_servers_wait_for_fresh_costmaps():
    assert params('controller_server.yaml', 'controller_server')['costmap_update_timeout'] > 0
    assert params('planner_server.yaml', 'planner_server')['costmap_update_timeout'] > 0


def bt_root():
    return ET.parse(BT_XML).getroot()


def test_bt_replans_on_timer_and_invalid_path():
    rate = bt_root().find('.//RateController')
    assert rate is not None and float(rate.get('hz')) >= 1.0
    assert rate.find('.//ComputePathToPose') is not None
    validate = rate.find('.//ValidatePath')
    assert validate is not None
    # Unknown != free (architecture §8.1): unknown cells on the path force a replan.
    assert validate.get('consider_unknown_as_obstacle') == 'true'
    assert rate.find('.//PathExpiringTimer') is not None


# ---------------------------------------------------------------- 4. BT + recoveries

def test_bt_nodes_exist_in_installed_nav2():
    catalogue = Path(get_package_share_directory('nav2_behavior_tree')) / 'nav2_tree_nodes.xml'
    nav2_ids = {el.get('ID') for el in ET.parse(catalogue).getroot().iter() if el.get('ID')}
    tags = {el.tag for el in bt_root().iter()} - {'root', 'BehaviorTree'}
    unknown = tags - nav2_ids - BT_BUILTINS
    assert not unknown, f'BT nodes not provided by Nav2/BT.CPP: {sorted(unknown)}'


def test_bt_recoveries_spin_wait_backup():
    recovery = bt_root().find('.//RoundRobin')
    assert {child.tag for child in recovery} == {'Spin', 'Wait', 'BackUp'}


def test_bt_never_clears_costmaps():
    clearing = [el.tag for el in bt_root().iter() if el.tag.startswith('ClearCostmap')
                or el.tag == 'ClearEntireCostmap']
    assert not clearing


def test_bt_ids_match_server_plugins():
    root = bt_root()
    planner = params('planner_server.yaml', 'planner_server')['planner_plugins']
    controller = params('controller_server.yaml', 'controller_server')['controller_plugins']
    assert root.find('.//PlannerSelector').get('default_planner') in planner
    assert root.find('.//ControllerSelector').get('default_controller') in controller
    p = params('bt_navigator.yaml', 'bt_navigator')
    # Lyrical bt_navigator throws on startup if the old name is present.
    assert 'error_code_names' not in p
    prefixes = p['error_code_name_prefixes']
    used = {el.get('error_code_id').strip('{}') for el in root.iter()
            if el.get('error_code_id')}
    assert used == {f'{prefix}_error_code' for prefix in prefixes}


def test_behavior_server_plugins():
    p = params('behavior_server.yaml', 'behavior_server')
    assert p['behavior_plugins'] == ['spin', 'backup', 'wait']
    assert p['spin']['plugin'] == 'nav2_behaviors::Spin'
    assert p['backup']['plugin'] == 'nav2_behaviors::BackUp'
    assert p['wait']['plugin'] == 'nav2_behaviors::Wait'


def test_navigate_to_pose_navigator():
    p = params('bt_navigator.yaml', 'bt_navigator')
    assert p['navigators'] == ['navigate_to_pose']
    assert p['navigate_to_pose']['plugin'] == 'nav2_bt_navigator::NavigateToPoseNavigator'
    assert p['global_frame'] == 'map' and p['robot_base_frame'] == 'base_link'


# ---------------------------------------------------------------- 5. cmd_vel boundary

def test_candidate_twist_is_unstamped():
    assert params('controller_server.yaml', 'controller_server')['enable_stamped_cmd_vel'] is False
    # Dev 5 contract: a zero twist when a goal ends.
    assert params('controller_server.yaml', 'controller_server')['publish_zero_velocity'] is True
    assert params('behavior_server.yaml', 'behavior_server')['enable_stamped_cmd_vel'] is False


def test_launch_remaps_every_cmd_vel_publisher_to_candidate():
    launch = load_launch_module()
    assert launch.CMD_VEL_REMAP == [('cmd_vel', '/cmd_vel_nav2')]
    specs = launch.nav2_specs('dev3.yaml', '', '', 'bt.xml', {}, False)
    publishers = {s['name'] for s in specs if s['remappings']}
    assert publishers == {'controller_server', 'behavior_server'}
    assert all(s['remappings'] == launch.CMD_VEL_REMAP for s in specs if s['remappings'])


def launch_context(**overrides):
    context = LaunchContext()
    context.launch_configurations.update({
        'costmap_params_file': str(FIXTURE_COSTMAPS), 'robot': '', 'robot_params_file': '',
        'footprint_file': '', 'bt_xml': str(BT_XML), 'use_sim_time': 'false',
        'autostart': 'false', 'log_level': 'info', 'use_composition': 'true', **overrides})
    return context


def test_launch_starts_only_dev4_nodes():
    specs = load_launch_module().nav2_specs('dev3.yaml', '', '', 'bt.xml', {}, False)
    assert {(s['package'], s['executable']) for s in specs} == {
        ('nav2_planner', 'planner_server'), ('nav2_controller', 'controller_server'),
        ('nav2_behaviors', 'behavior_server'), ('nav2_bt_navigator', 'bt_navigator'),
        ('nav2_lifecycle_manager', 'lifecycle_manager'),
        ('ugv_navigation', 'nav2_heartbeat')}


def launched(launch=None, **overrides):
    """(containers, standalone nodes) that _launch_setup returns."""
    from launch_ros.actions import ComposableNodeContainer, Node
    actions = (launch or load_launch_module())._launch_setup(launch_context(**overrides))
    return ([a for a in actions if isinstance(a, ComposableNodeContainer)],
            [a for a in actions if isinstance(a, Node)
             and not isinstance(a, ComposableNodeContainer)])


def test_composition_puts_nav2_in_one_container_and_keeps_heartbeat_separate():
    containers, nodes = launched()
    assert len(containers) == 1
    # Heartbeat must survive a container crash, so it is never composed.
    assert [n.node_executable for n in nodes] == ['nav2_heartbeat']
    specs = load_launch_module().nav2_specs('dev3.yaml', '', '', 'bt.xml', {}, False)
    assert [s['name'] for s in specs if s['plugin'] is None] == ['nav2_heartbeat']


def test_container_gets_costmap_params_for_child_costmap_nodes():
    # Costmaps are child nodes of planner/controller; they read the process's
    # --params-file, so the container needs Dev 3's costmap file (+ footprint, robot).
    launch = load_launch_module()
    specs = launch.nav2_specs('dev3.yaml', 'robot.yaml', 'fp.yaml', 'bt.xml', {}, False)
    files = launch.container_parameter_files([s for s in specs if s['plugin']])
    defaults = [str(CONFIG / f) for f in DEV4_YAMLS]
    assert sorted(files[:4]) == sorted(defaults)
    assert files[4:] == ['dev3.yaml', 'fp.yaml', 'robot.yaml']


def test_without_composition_every_node_is_a_process():
    containers, nodes = launched(use_composition='false')
    assert not containers and len(nodes) == 6


def test_launch_requires_dev3_costmap_params():
    launch = load_launch_module()
    launch.DEFAULT_COSTMAP_PARAMS = PKG / 'test' / 'fixtures' / 'does_not_exist.yaml'
    with pytest.raises(RuntimeError, match='Dev 3'):
        launch._launch_setup(launch_context(costmap_params_file=''))


def test_launch_uses_dev3_default_costmap_params_when_present():
    launch = load_launch_module()
    launch.DEFAULT_COSTMAP_PARAMS = FIXTURE_COSTMAPS  # stands in for Dev 3's file
    containers, nodes = launched(launch, costmap_params_file='')
    assert len(containers) == 1 and len(nodes) == 1


# ---------------------------------------------------------------- per-robot overlays

ROBOTS = sorted(d.name for d in (CONFIG / 'robots').iterdir() if d.is_dir())


def robot_limit_keys():
    """{node: {dotted key}} for every value marked [ROBOT LIMIT] in the Dev 4 defaults."""
    marked = {}
    for file_name, node in (('controller_server.yaml', 'controller_server'),
                            ('behavior_server.yaml', 'behavior_server')):
        stack = []
        for line in (CONFIG / file_name).read_text().splitlines():
            m = re.match(r'^( *)([A-Za-z_]+):', line)
            if not m:
                continue
            depth = len(m.group(1)) // 2
            stack = stack[:depth] + [m.group(2)]
            if '[ROBOT LIMIT]' in line:
                marked.setdefault(node, set()).add('.'.join(stack[2:]))
    return marked


def flatten(d, prefix=''):
    for k, v in d.items():
        if isinstance(v, dict):
            yield from flatten(v, f'{prefix}{k}.')
        else:
            yield f'{prefix}{k}', v


def test_there_are_two_robot_overlays():
    # architecture §13 item 9: a second robot / footprint.
    assert {'primary', 'secondary'} <= set(ROBOTS)


@pytest.mark.parametrize('robot', ROBOTS)
def test_robot_overlay_covers_exactly_the_robot_limits(robot):
    data = yaml.safe_load((CONFIG / 'robots' / robot / 'nav2_limits.yaml').read_text())
    marked = robot_limit_keys()
    assert set(data) == set(marked)
    for node, keys in marked.items():
        assert set(dict(flatten(data[node]['ros__parameters']))) == keys, node


@pytest.mark.parametrize('robot', ROBOTS)
def test_robot_overlay_limits_are_consistent(robot):
    data = yaml.safe_load((CONFIG / 'robots' / robot / 'nav2_limits.yaml').read_text())
    r = data['controller_server']['ros__parameters']['FollowPath']
    b = data['behavior_server']['ros__parameters']
    assert r['max_linear_vel'] > 0
    assert r['min_angular_vel'] == -r['max_angular_vel'] < 0
    assert r['max_linear_accel'] > 0 > r['max_linear_decel']
    assert r['max_angular_accel'] > 0 > r['max_angular_decel']
    assert 0 < r['rotate_to_heading_angular_vel'] <= r['max_angular_vel']
    assert r['cancel_deceleration'] > 0
    assert 0 < b['min_rotational_vel'] <= b['max_rotational_vel']
    assert b['rotational_acc_lim'] > 0


@pytest.mark.parametrize('robot', ROBOTS)
def test_launch_robot_arg_loads_overlay_last(robot):
    launch = load_launch_module()
    overlay = launch.resolve_robot_params(robot, '')
    assert overlay == str(CONFIG / 'robots' / robot / 'nav2_limits.yaml')
    # Dev 4 defaults < Dev 3 costmaps < robot overlay
    assert launch.parameter_files('controller_server.yaml', 'dev3.yaml', overlay) == [
        str(CONFIG / 'controller_server.yaml'), 'dev3.yaml', overlay]
    specs = launch.nav2_specs('dev3.yaml', overlay, '', 'bt.xml', {}, False)
    for s in specs[:4]:  # every Nav2 server gets the overlay as its last file
        assert [p for p in s['parameters'] if isinstance(p, str)][-1] == overlay
    containers, _ = launched(launch, robot=robot)
    assert len(containers) == 1


FIXTURE_FOOTPRINT = PKG / 'test' / 'fixtures' / 'test_only_footprint_primary.yaml'


def test_footprint_file_becomes_costmap_params_for_both_costmaps():
    launch = load_launch_module()
    footprint = launch.load_footprint(FIXTURE_FOOTPRINT)
    assert yaml.safe_load(footprint['footprint'])[0] == [0.30, 0.22]
    data = yaml.safe_load(Path(launch.footprint_params_file(footprint)).read_text())
    for costmap in ('global_costmap', 'local_costmap'):
        assert data[costmap][costmap]['ros__parameters'] == footprint
    # Dev 4 defaults < Dev 3 costmaps < footprint < robot limits
    assert launch.parameter_files('planner_server.yaml', 'dev3.yaml', 'robot.yaml',
                                  'fp.yaml')[1:] == ['dev3.yaml', 'fp.yaml', 'robot.yaml']


def test_footprint_only_goes_to_costmap_hosts():
    launch = load_launch_module()
    # planner_server hosts global_costmap, controller_server hosts local_costmap.
    assert launch.COSTMAP_HOSTS == {'planner_server': 'global_costmap',
                                    'controller_server': 'local_costmap'}
    specs = launch.nav2_specs('dev3.yaml', '', 'fp.yaml', 'bt.xml', {}, False)
    with_fp = {s['name'] for s in specs if 'fp.yaml' in s['parameters']}
    assert with_fp == set(launch.COSTMAP_HOSTS)


def test_footprint_params_file_is_reused_not_leaked():
    launch = load_launch_module()
    footprint = launch.load_footprint(FIXTURE_FOOTPRINT)
    assert launch.footprint_params_file(footprint) == launch.footprint_params_file(footprint)


def test_footprint_search_stops_at_repo_root(tmp_path):
    launch = load_launch_module()
    (tmp_path / 'config' / 'robots').mkdir(parents=True)
    (tmp_path / 'config' / 'robots' / 'footprint_primary.yaml').write_text(
        FIXTURE_FOOTPRINT.read_text())
    repo = tmp_path / 'repo'
    (repo / '.git').mkdir(parents=True)
    pkg = repo / 'src' / 'ugv_navigation'
    pkg.mkdir(parents=True)
    # A footprint above the repository root must not be picked up.
    assert launch.find_footprint_file('primary', pkg) == ''


def test_robot_arg_finds_dev5_footprint_above_package(tmp_path):
    launch = load_launch_module()
    pkg = tmp_path / 'src' / 'ugv_navigation'
    pkg.mkdir(parents=True)
    assert launch.find_footprint_file('primary', pkg) == ''
    (tmp_path / 'config' / 'robots').mkdir(parents=True)
    dev5 = tmp_path / 'config' / 'robots' / 'footprint_primary.yaml'
    dev5.write_text(FIXTURE_FOOTPRINT.read_text())
    assert launch.find_footprint_file('primary', pkg) == str(dev5)


def test_bad_footprint_is_rejected(tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text('footprint: "[[0.3, 0.2], [0.3]]"\n')
    with pytest.raises(RuntimeError, match='polygon'):
        load_launch_module().load_footprint(bad)


def test_launch_rejects_unknown_robot_and_double_overlay():
    launch = load_launch_module()
    with pytest.raises(RuntimeError, match='unknown robot'):
        launch._launch_setup(launch_context(robot='no_such_robot'))
    with pytest.raises(RuntimeError, match='not both'):
        launch._launch_setup(launch_context(robot='primary',
                                            robot_params_file=str(FIXTURE_COSTMAPS)))


# ---------------------------------------------------------------- ownership boundary

@pytest.mark.parametrize('file_name', DEV4_YAMLS + [
    f'robots/{robot}/nav2_limits.yaml' for robot in ROBOTS])
def test_dev4_config_holds_no_costmap_or_footprint(file_name):
    with open(CONFIG / file_name) as f:
        data = yaml.safe_load(f)
    assert not {'global_costmap', 'local_costmap'} & set(data)

    def keys(d):
        for k, v in d.items():
            yield k
            if isinstance(v, dict):
                yield from keys(v)
    assert not {'footprint', 'robot_radius', 'plugins'} & set(keys(data))
