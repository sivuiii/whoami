"""
ROS-free logic for the Nav2 heartbeat. Unit-testable without a graph.

Architecture §12: "Nav2 crash / no controller heartbeat -> hold".
"""

from dataclasses import dataclass, field

HEARTBEAT_TOPIC = '/ugv/nav2_heartbeat'
STATUS_TOPIC = '/ugv/nav2_status'
DEFAULT_SERVERS = ('planner_server', 'controller_server', 'behavior_server', 'bt_navigator')


@dataclass
class HeartbeatMonitor:
    """
    Tracks the last time each Nav2 server answered get_state with 'active'.

    Healthy only if every server's latest reply is 'active' and arrived within
    `max_age_s`. An inactive, crashed, hung or never-seen server makes it unhealthy
    (fail closed).
    """

    servers: tuple = DEFAULT_SERVERS
    max_age_s: float = 0.5
    last_active_s: dict = field(default_factory=dict)
    last_state: dict = field(default_factory=dict)

    def record(self, server, state_label, now_s):
        """Store one get_state reply for `server`."""
        self.last_state[server] = state_label
        if state_label == 'active':
            self.last_active_s[server] = now_s

    def evaluate(self, now_s, check_age=True):
        """
        Return (healthy, reason). `reason` is 'ok' or names the first failing server.

        check_age=False skips the staleness check (explicit non-active replies still
        fail): used right after the monitoring process itself was not scheduled, when
        old reply times say nothing about the servers.
        """
        for server in self.servers:
            state = self.last_state.get(server, 'no reply')
            seen = self.last_active_s.get(server)
            # An explicit non-active reply fails at once; silence fails after max_age.
            if state != 'active' or seen is None:
                return False, f'{server} not active ({state})'
            if check_age and now_s - seen > self.max_age_s:
                return False, f'{server} stale (no reply for {now_s - seen:.2f} s)'
        return True, 'ok'
