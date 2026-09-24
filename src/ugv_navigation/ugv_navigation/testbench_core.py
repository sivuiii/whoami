"""ROS-free logic for the Dev 4 navigation testbench (unit-testable without a graph)."""

from dataclasses import dataclass, field
import math

CANDIDATE_TOPIC = '/cmd_vel_nav2'
FINAL_TOPIC = '/cmd_vel'
NAVIGATE_ACTION = '/navigate_to_pose'
TWIST_TYPE = 'geometry_msgs/msg/Twist'

# Nodes started by navigation.launch.py, including the costmap sub-nodes.
DEV4_NODES = frozenset({
    'planner_server', 'controller_server', 'behavior_server', 'bt_navigator',
    'lifecycle_manager_navigation', 'global_costmap', 'local_costmap',
})
LIFECYCLE_NODES = ('planner_server', 'controller_server', 'behavior_server', 'bt_navigator')


@dataclass(frozen=True)
class Endpoint:
    node: str
    namespace: str
    topic_type: str


@dataclass
class BoundaryReport:
    ok: bool = True
    errors: list = field(default_factory=list)
    warnings: list = field(default_factory=list)
    info: list = field(default_factory=list)

    def fail(self, msg):
        self.ok = False
        self.errors.append(msg)


def evaluate_boundary(candidate_publishers, final_publishers):
    """
    Check the architecture §3.1 boundary from graph publisher info.

    candidate_publishers / final_publishers: iterables of Endpoint on
    /cmd_vel_nav2 and /cmd_vel respectively.
    """
    report = BoundaryReport()
    # One node can own several publishers (each behavior plugin has its own).
    candidate_publishers = sorted(set(candidate_publishers), key=lambda ep: ep.node)
    final_publishers = sorted(set(final_publishers), key=lambda ep: ep.node)

    for ep in final_publishers:
        if ep.node in DEV4_NODES:
            report.fail(f'Dev 4 node "{ep.node}" publishes {FINAL_TOPIC} '
                        f'(only Dev 5 safety authority may)')
        else:
            report.info.append(f'{FINAL_TOPIC} publisher "{ep.node}" is external '
                               f'(expected: Dev 5 safety authority)')

    if not candidate_publishers:
        report.warnings.append(f'no publisher on {CANDIDATE_TOPIC} yet '
                               f'(Nav2 servers not configured?)')
    for ep in candidate_publishers:
        if ep.topic_type != TWIST_TYPE:
            report.fail(f'{CANDIDATE_TOPIC} from "{ep.node}" has type {ep.topic_type}, '
                        f'contract is {TWIST_TYPE}')
        if ep.node not in DEV4_NODES:
            report.warnings.append(f'unexpected non-Dev 4 publisher on {CANDIDATE_TOPIC}: '
                                   f'"{ep.node}"')
        else:
            report.info.append(f'{CANDIDATE_TOPIC} publisher "{ep.node}" ({ep.topic_type})')
    return report


def parse_goal(text):
    """Parse 'x,y' or 'x,y,yaw' (metres, radians) into a (x, y, yaw) tuple."""
    parts = [p.strip() for p in text.split(',')]
    if len(parts) not in (2, 3) or not all(parts):
        raise ValueError(f'goal must be "x,y" or "x,y,yaw", got "{text}"')
    values = [float(p) for p in parts]
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f'goal values must be finite, got "{text}"')
    x, y = values[0], values[1]
    yaw = values[2] if len(values) == 3 else 0.0
    return x, y, yaw


def yaw_to_quaternion(yaw):
    """Planar yaw (rad) -> quaternion (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


@dataclass
class TwistStats:
    """Running statistics of candidate twists seen on /cmd_vel_nav2."""

    count: int = 0
    moving: int = 0
    max_abs_linear: float = 0.0
    max_abs_angular: float = 0.0
    min_linear: float = 0.0

    def add(self, linear_x, angular_z):
        self.count += 1
        if abs(linear_x) > 1e-6 or abs(angular_z) > 1e-6:
            self.moving += 1
        self.max_abs_linear = max(self.max_abs_linear, abs(linear_x))
        self.max_abs_angular = max(self.max_abs_angular, abs(angular_z))
        self.min_linear = min(self.min_linear, linear_x)


@dataclass
class GoalRun:
    goal: tuple
    status: str
    elapsed_s: float
    recoveries: int
    error_code: int
    error_msg: str
    twist: TwistStats


def format_summary(runs):
    header = (f'{"#":>2}  {"goal (x, y, yaw)":<24} {"status":<10} {"time[s]":>8} '
              f'{"recov":>5} {"err":>5} {"cmd msgs":>8} {"max|v|":>7} {"max|w|":>7}')
    lines = [header, '-' * len(header)]
    for i, run in enumerate(runs, 1):
        x, y, yaw = run.goal
        lines.append(
            f'{i:>2}  {f"({x:.2f}, {y:.2f}, {yaw:.2f})":<24} {run.status:<10} '
            f'{run.elapsed_s:>8.2f} {run.recoveries:>5} {run.error_code:>5} '
            f'{run.twist.count:>8} {run.twist.max_abs_linear:>7.3f} '
            f'{run.twist.max_abs_angular:>7.3f}')
    return '\n'.join(lines)
