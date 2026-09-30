# Dev 4 interfaces: hand-off to Dev 2, Dev 3 and Dev 5

What the Dev 4 planning/control stack (`ugv_navigation`, `navigation.launch.py`)
consumes and produces, and what it needs decided by the other devs. Companion to
Dev 3's `ugv_navigation/DEV3_DEV4_INTERFACE.md`. `architecture.md` wins on conflict.

Labels: **IMPLEMENTED** (in this package, tested on ROS 2 Lyrical / Nav2 1.5.1) ·
**PROPOSED** (Dev 4 proposal, needs the other dev's agreement) · **OPEN** (not decided).

Evidence: `src/ugv_navigation/docker/test_in_lyrical.sh src/ugv_navigation` runs every test below.

## 1. Topics and actions

| Name | Type | Direction | Status | Contract |
|---|---|---|---|---|
| `/cmd_vel_nav2` | `geometry_msgs/msg/Twist` (unstamped) | Dev 4 → Dev 5 | IMPLEMENTED | Candidate only (§3.1). Published by `controller_server` (~20 Hz while following a path) and `behavior_server` (spin / backup recoveries). One zero twist when a goal ends. **Silent when idle and during the Wait recovery.** Bounds: \|v\| ≤ 0.4 m/s, \|ω\| ≤ 0.8 rad/s (placeholders, §5); v < 0 only during BackUp (0.10 m/s, 0.30 m) |
| `/cmd_vel` | — | — | IMPLEMENTED | **No Dev 4 node publishes it.** Checked live by `test_navigation_graph`, `test_closed_loop` and `nav_goal_testbench check` |
| `/ugv/nav2_heartbeat` | `std_msgs/msg/Bool` | Dev 4 → Dev 5 | IMPLEMENTED | 20 Hz, every tick. `true` only if planner, controller, behavior server and bt_navigator all answered lifecycle `get_state` = `active` within 0.5 s. `false` at startup, within one poll (0.2 s) of any server reporting non-active, and within 0.5 s + one poll (0.1 s) of one crashing or hanging. A single lost `get_state` reply is retried after 0.2 s, so it never makes a healthy server stale. If the heartbeat process itself is starved of CPU, it does not blame Nav2 for its own pause (age check resumes after one fresh poll round). Verified: 15/15 live runs with no false `false`, and `false` after a killed server. Same shape as Dev 2's `/ugv/pose_valid` |
| `/ugv/nav2_status` | `std_msgs/msg/String`, transient local | Dev 4 → any | IMPLEMENTED | `ok` or the reason, e.g. `controller_server stale (no reply for 0.61 s)`. Published on change |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | operator / mission → Dev 4 | IMPLEMENTED | Goals in `map`. Result `error_code` / `error_msg` from `compute_path`, `follow_path`, `spin`, `wait`, `backup` |
| TF `map → odom → base_link`, `/odom` | TF, `nav_msgs/msg/Odometry` | Dev 2 → Dev 4 | IMPLEMENTED (consumer) | Matches Dev 2's `odom_selector` output. Nav2 activation waits for the TF |
| Global / local costmaps | Nav2 costmap layers | Dev 3 → Dev 4 | **PROPOSED** (§3) | Dev 3's params file, `costmap_params_file:=` or `config/costmaps.yaml` |

## 2. For Dev 5 (safety authority, bringup, platform)

**Watchdog.** Use `/ugv/nav2_heartbeat` for the §12 "Nav2 crash / no controller
heartbeat" row: hold on `false` **or** on message age > your timeout (the heartbeat
node itself can die). Do **not** use `/cmd_vel_nav2` age as Nav2 liveness: it is
silent whenever no goal is running and for 5 s during the Wait recovery. Keep a
separate freshness check on the candidate: a `/cmd_vel_nav2` older than ~0.5 s must be
treated as zero at Level 4 (never repeat the last candidate). That is a candidate
property, not a health fault.

**Bringup include** (IMPLEMENTED on our side):

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('ugv_navigation'), 'launch', 'navigation.launch.py'])),
    launch_arguments={
        'costmap_params_file': <Dev 3 costmap yaml>,   # or Dev 3's config/costmaps.yaml
        'robot': 'primary',                            # or 'secondary', see §5
        'use_sim_time': 'true',                        # profile:=sim only
    }.items())
```

Launches only: `planner_server`, `controller_server`, `behavior_server`,
`bt_navigator`, `lifecycle_manager_navigation` (by default composed into one
`nav2_container` process; `use_composition:=false` for separate processes) and
`nav2_heartbeat` (always its own process, so it reports `false` if the container dies).
Node, topic and service names are the same in both modes. No camera, TF, map server,
velocity smoother, collision monitor or motor driver.

**Robot limits** (OPEN, Dev 5 values needed). Fill
`config/robots/{primary,secondary}/nav2_limits.yaml` per platform: max linear speed,
max angular speed, linear accel / decel, angular accel / decel, rotate-in-place speed,
cancel deceleration, recovery spin speeds. Current values are placeholders
(0.4 m/s, 0.8 rad/s, 0.5 / -1.0 m/s², ±2.0 rad/s²). A test checks each file
overrides exactly the `[ROBOT LIMIT]` keys and that the limits are consistent.

**Safety hold vs. navigation** (OPEN, decision needed). When you hold `/cmd_vel` at
zero (perception degraded, pose invalid), Nav2 does not know: after 10 s without
0.5 m of progress the controller fails, spin / wait / backup run (also held), and
after 6 retries the goal is ABORTED. Options:

| Option | Behaviour | Needs |
|---|---|---|
| A. Accept abort | Operator / mission re-sends the goal after the hold | Nothing |
| B. Hold-aware BT (Dev 4 recommends) | BT pauses (no progress check, no recoveries) while a hold is active, resumes the same goal after | Dev 5 publishes e.g. `/ugv/safety_hold` (`std_msgs/Bool`, 20 Hz); Dev 4 adds the BT condition |
| C. Longer progress timeout | Tolerates short holds only | Nothing; masks real "stuck" cases |

## 3. For Dev 3 (costmaps)

**Feeding Dev 3's grid to Nav2** (PROPOSED; works end to end in the closed-loop
harness). Nav2's planner and controller cannot subscribe to an external costmap topic;
they host `global_costmap` / `local_costmap` and those publish
`/global_costmap/costmap` and `/local_costmap/costmap` themselves. So:

1. Dev 3 publishes its fused grid as `nav_msgs/OccupancyGrid` on its **own** topic
   (proposal: `/ugv/costmap_grid`, frame `map`, transient local), updating as the
   live mask / voxel data changes. Not `/map`: RTAB-Map (Dev 2) already publishes
   its own occupancy grid there.
   **Publish it un-inflated** (free / hazard-or-geometry lethal / unknown, plus any
   deliberate semantic soft cost). Inflation must happen exactly once, in Nav2's
   `InflationLayer` (step 3): it uses the robot footprint, `inflate_around_unknown`,
   and the exponential decay RPP's cost-regulated speed assumes. Inflating in
   `costmap_core` as well would stack two margins and make the robot overly cautious.
2. Values: free `0` → 0 · lethal / hazard / geometry `254` → **100** · unknown `255`
   → **-1** · inscribed `253` → **99** (StaticLayer's `inscribed_obstacle_cost_value`)
   · intermediate `1..252` → `max(1, round(c * 98 / 252))` (StaticLayer scales
   `v / 100 * 254` back).
3. Both Nav2 costmaps set `track_unknown_space: true` and `trinary_costmap: false`
   (costmap-level parameters; the default `trinary_costmap: true` turns every
   intermediate cost into free), read the grid with `nav2_costmap_2d::StaticLayer`
   (`map_topic: /ugv/costmap_grid`) and add `nav2_costmap_2d::InflationLayer`.
   The rolling `odom` local costmap works with a `map`-frame grid (StaticLayer
   transforms per cell).

Template: `test/fixtures/test_only_closed_loop_costmaps.yaml` (values are test values).

**Required costmap settings** (for Smac2D / RPP to behave as designed):

| Setting | Why |
|---|---|
| Inflation `inflate_around_unknown: true` (both costmaps) | Architecture §8.1 "unknown = never free — inflate". Without it, in the `unknown_block` scenario the robot cut a corner, ended with its centre in unknown cells and Smac2D (`allow_unknown: false`) aborted with *"Start occupied"*. With it the robot kept 0.50 m from unknown |
| Inflation `inflation_radius` ≥ half the robot's largest cross-section | Smac2D collision checking; otherwise it logs *"inflation is not set sufficiently"* |
| Local inflation `cost_scaling_factor` = RPP `inflation_cost_scaling_factor` (3.0) | RPP recovers obstacle distance from cost; tell Dev 4 if you change it |
| Local costmap update ≥ 5 Hz; global ≥ 1 Hz | How fast new hazards reach RPP / the 2 Hz replanning |
| `footprint` or `robot_radius` in both costmaps | Smac2D 2D plans a circle (inflation); RPP collision checks the footprint |

**Unknown-space policy** (OPEN). With the settings above the robot never plans
through unknown cells and keeps an inflation margin from them. If Dev 3 marks every
cell the camera has not yet seen as unknown, the planner cannot reach goals beyond
what has been observed. Options: mark unobserved space differently from class-0
pixels (e.g. unobserved = free or a moderate cost in the global grid, class 0 =
unknown), or Dev 4 enables `allow_unknown` in Smac2D with unknown as a high cost.
Needs a joint decision against architecture §8.1.

**Package location** (OPEN). Dev 3's `costmap_core` is at the repo root
`ugv_navigation/`; this package is `src/ugv_navigation/`; architecture §7 says
`ugv_nav/ugv_navigation/`. Both devs own one ROS package; agree on one place.

## 4. Cross-dev findings (not Dev 4's to fix)

**TF: `base_link` would have two parents (Dev 2 × Dev 5).** Dev 5's URDF
(`ugv_robot_description/urdf/ugv.urdf.xacro`, #15) makes `base_footprint` the parent of
`base_link`; Dev 2's `odom_selector` publishes `odom → base_link`. With
`robot_state_publisher` running, `base_link` gets two parents, which TF does not allow
(lookups flip between trees, Nav2 sees jumps or `TF_OLD_DATA`). Either Dev 2 publishes
`odom → base_footprint` (and Nav2 uses `base_footprint`), or the URDF roots at
`base_link`. Dev 4 follows the documented contract `map → odom → base_link`
(dev.md §3) and changes `robot_base_frame` only if that contract changes.

## 5. Per-robot configuration

`ros2 launch ugv_navigation navigation.launch.py robot:=<name>` selects, per robot:

| What | File | Owner | Applied to |
|---|---|---|---|
| Speed / acceleration limits | `config/robots/<name>/nav2_limits.yaml` (this package) | Dev 4 file, Dev 5 values | controller + behavior server |
| Footprint (`footprint`, `footprint_padding`) | Dev 5's `config/robots/footprint_<name>.yaml`, found by searching upward from this package (source tree, or an install space built inside the repo) | Dev 5 (selection is Dev 4's, per Dev 5) | both costmaps |

Parameter order: Dev 4 defaults < Dev 3 costmap params < footprint < robot limits.
`footprint_file:=<path>` overrides the footprint lookup; `robot_params_file:=<path>`
replaces the limits file (not together with `robot:=`). Without either, the footprint
is whatever Dev 3's costmap params set. The closed-loop tests run every scenario with
both of Dev 5's footprints (TEST-ONLY copies from #15 until it is merged).

## 6. Evidence (Lyrical, Nav2 1.5.1)

| Test | Proves |
|---|---|
| `test_dev4_config` | Every plugin / parameter / BT node exists in the installed Nav2; contracts above |
| `test_navigation_graph` | Full stack activates; only controller + behavior server publish `/cmd_vel_nav2`; nothing publishes `/cmd_vel`; heartbeat `false` while configured, `true` when active, `false` after `controller_server` is killed |
| `test_closed_loop` | A→B in 5 scenarios (open, wall with opening, corridor, unknown block, hazard appearing mid-run) × Dev 5's 2 footprints, TEST-ONLY fake base: goal reached, footprint polygon clear of lethal and unknown cells, twists within limits. Benchmark CSV in the build dir; numbers in `README.md` |
| `test_heartbeat_core`, `test_testbench_core` | Heartbeat and testbench logic |
