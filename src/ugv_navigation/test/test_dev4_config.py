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
    header = (Path(get_package_prefix('nav2_regulated_pure_pursuit_controller')) / 'include'
              / 'nav2_regulated_pure_pursuit_controller' / 'parameter_handler.hpp')
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
    assert 0 < r['regulated_linear_scaling_min_speed'] < r['desired_linear_vel']


def test_rpp_differential_drive():
    r = params('controller_server.yaml', 'controller_server')['FollowPath']
    assert r['use_rotate_to_heading'] is True
    # RPP rejects rotate-to-heading together with reversing.
    assert r['allow_reversing'] is False


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
    assert rate.find('.//IsPathValid') is not None
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
    names = params('bt_navigator.yaml', 'bt_navigator')['error_code_names']
    used = {el.get('error_code_id').strip('{}') for el in root.iter()
            if el.get('error_code_id')}
    assert used <= set(names)


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
    assert params('behavior_server.yaml', 'behavior_server')['enable_stamped_cmd_vel'] is False


def test_launch_remaps_every_cmd_vel_publisher_to_candidate():
    launch = load_launch_module()
    assert launch.CMD_VEL_REMAP == [('cmd_vel', '/cmd_vel_nav2')]
    publishers = {name for _, _, name, _, pub in launch.NAV2_SERVERS if pub}
    assert publishers == {'controller_server', 'behavior_server'}


def test_launch_starts_only_dev4_nodes():
    launch = load_launch_module()
    context = LaunchContext()
    context.launch_configurations.update({
        'costmap_params_file': str(FIXTURE_COSTMAPS), 'robot_params_file': '',
        'bt_xml': str(BT_XML), 'use_sim_time': 'false', 'autostart': 'false',
        'log_level': 'info'})
    nodes = launch._launch_setup(context)
    assert {(n.node_package, n.node_executable) for n in nodes} == {
        ('nav2_planner', 'planner_server'), ('nav2_controller', 'controller_server'),
        ('nav2_behaviors', 'behavior_server'), ('nav2_bt_navigator', 'bt_navigator'),
        ('nav2_lifecycle_manager', 'lifecycle_manager')}


def test_launch_requires_dev3_costmap_params():
    launch = load_launch_module()
    context = LaunchContext()
    context.launch_configurations.update({
        'costmap_params_file': '', 'robot_params_file': '', 'bt_xml': str(BT_XML),
        'use_sim_time': 'false', 'autostart': 'false', 'log_level': 'info'})
    launch.DEFAULT_COSTMAP_PARAMS = PKG / 'test' / 'fixtures' / 'does_not_exist.yaml'
    with pytest.raises(RuntimeError, match='Dev 3'):
        launch._launch_setup(context)


def test_launch_uses_dev3_default_costmap_params_when_present():
    launch = load_launch_module()
    launch.DEFAULT_COSTMAP_PARAMS = FIXTURE_COSTMAPS  # stands in for Dev 3's file
    context = LaunchContext()
    context.launch_configurations.update({
        'costmap_params_file': '', 'robot_params_file': '', 'bt_xml': str(BT_XML),
        'use_sim_time': 'false', 'autostart': 'false', 'log_level': 'info'})
    nodes = launch._launch_setup(context)
    assert len(nodes) == 5


# ---------------------------------------------------------------- ownership boundary

@pytest.mark.parametrize('file_name', DEV4_YAMLS)
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
