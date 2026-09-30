# ugv_navigation — Dev 4: Planning & Trajectory Control

Nav2 planning/control core (architecture.md §3.1, §6, §11; dev.md Dev 4).
The costmap subsystem in this package belongs to Dev 3 and is **not** configured here.
Interfaces and open decisions for Dev 2 / 3 / 5: [`DEV4_INTERFACES.md`](DEV4_INTERFACES.md).

```
Dev 3 costmap params ─► global_costmap (in planner_server)  ─► Smac2D ─┐
Dev 2 TF map→odom→base_link, /odom                                     │ path
Dev 3 costmap params ─► local_costmap  (in controller_server) ─► RPP ◄─┘
                                                               │
                  bt_navigator (/navigate_to_pose, replanning + recoveries)
                                                               │
       controller_server + behavior_server  cmd_vel ──remap──► /cmd_vel_nav2 ─► Dev 5 ─► /cmd_vel
```

| File | Dev 4 task |
|---|---|
| `config/planner_server.yaml` | 1 Smac2D (`allow_unknown: false`, bounded plan time) |
| `config/controller_server.yaml` | 2 RPP with Dynamic Window (velocity + acceleration limits enforced), adaptive lookahead, curvature + cost regulation, collision detection, rotate-to-heading · 3 fresh-costmap wait · 5 unstamped `Twist` |
| `behavior_trees/navigate_to_pose_ugv.xml` | 3 replan at 2 Hz if path > 2 s old / goal updated / `ValidatePath` fails on the path ahead (unknown counts as obstacle) · 4 spin / wait / backup |
| `config/behavior_server.yaml` | 4 recovery plugins (spin, backup, wait) |
| `config/bt_navigator.yaml` | 4 `/navigate_to_pose` |
| `launch/navigation.launch.py` | all; 5 remaps every Nav2 `cmd_vel` to `/cmd_vel_nav2`; `robot:=<name>` selects limits + Dev 5's footprint |
| `scripts/nav2_heartbeat`, `ugv_navigation/heartbeat_core.py` | `/ugv/nav2_heartbeat` (20 Hz, fail closed) for Dev 5's §12 watchdog |
| `config/robots/{primary,secondary}/nav2_limits.yaml` | per-robot `[ROBOT LIMIT]` overlays (placeholders until Dev 5's limits) |
| `scripts/nav_goal_testbench`, `ugv_navigation/testbench_core.py` | 6 CLI / goal benchmark |
| `test/` | static contract tests, testbench unit tests, live graph boundary test |
| `test/closed_loop/`, `test/test_closed_loop.py` | 2 · 3 · 6 closed-loop A→B scenarios + benchmark CSV (TEST-ONLY fake base and maps) |

## Design decisions

- **No costmap clearing in the BT.** `ClearEntireCostmap` would drop Dev 3's hazard,
  unknown and fail-safe ROI cost until the next update, which would briefly treat
  those cells as free (§8.1, §8.6, §9). Recoveries are spin, wait and backup only.
- **Recovery motion is candidate motion too.** `behavior_server`'s `cmd_vel` is also
  remapped to `/cmd_vel_nav2`, so spin and backup go through Dev 5.
- **No velocity smoother or collision monitor.** Deceleration and the final gate are
  Dev 5's.
- **No footprint and no costmap values** are set in any Dev 4 file. Smac2D and RPP
  read the footprint and inflation from Dev 3's costmaps.
- **Composition by default.** The four Nav2 servers and the lifecycle manager run as
  components in one `nav2_container` (`use_composition:=false` for separate processes).
  Measured on Lyrical, same A→B goal, fresh container each: **120 MB vs 315 MB RSS
  (−62 %), 4.8 s vs 7.0 s CPU (−31 %)**, identical path and goal time. The container
  also gets every parameter file: the costmaps are child nodes that read their
  sections from the process's `--params-file` arguments. `nav2_heartbeat` is never
  composed, so it still reports `false` if the container dies.
- **Target distro is ROS 2 Lyrical** (`CLAUDE.md`). Configs use Lyrical Nav2 names
  (`error_code_name_prefixes`, `ValidatePath`, RPP `max_linear_vel`, controller
  `path_handler_plugins`) and will not load on Jazzy Nav2. The static tests check
  every name against the installed Nav2, so run them on Lyrical.

## Closed-loop benchmark

`test_closed_loop` drives the real launch file against a TEST-ONLY kinematic base
(integrates `/cmd_vel_nav2`, publishes `/odom` + `odom→base_link`; stands in for Dev 2
and Dev 5) and synthetic maps (stand in for Dev 3), once per scenario for each of Dev 5's
footprints (primary 0.60 × 0.44 m + 0.02 m padding, secondary 0.48 × 0.36 m + 0.01 m;
test copies from #15). It asserts goal reached, footprint polygon never on lethal or
unknown cells, candidate twists inside the RPP limits, the mid-run hazard avoided, and
no `/cmd_vel` publisher. Results go to `build/ugv_navigation/closed_loop_benchmark.csv`.
Lyrical, placeholder limits (0.4 m/s, 0.8 rad/s); clearance = physical footprint to
nearest cell centre:

| Scenario | Robot | Time | Path | Min clearance | Recoveries |
|---|---|---|---|---|---|
| open | primary / secondary | 10.2 s / 10.2 s | 3.76 m | – | 0 |
| wall_gap (0.8 m opening) | primary / secondary | 15.1 s / 15.1 s | 4.77 / 4.75 m | 0.14 / 0.19 m to lethal | 0 |
| corridor (1.2 m) | primary / secondary | 10.4 s / 10.2 s | 3.76 m | 0.41 / 0.45 m to lethal | 0 |
| unknown_block | primary / secondary | 16.0 s / 16.2 s | 5.12 m | 0.28 / 0.32 m to unknown | 0 |
| dynamic_obstacle (appears at x ≥ 0.8 m) | primary / secondary | 15.3 s / 14.9 s | 4.73 m | 0.26 / 0.30 m to lethal | 0 |

The 0.8 m opening is the tightest case: the primary robot passes with 0.14 m to spare.

This proves the planning/control logic, not the robot: no perception, localization
drift, Dev 3 costmaps, Dev 5 safety gate or real dynamics. Retune on the platform.

## Values to confirm with other devs

| Key | Owner | Why |
|---|---|---|
| `[ROBOT LIMIT]` values in `controller_server.yaml` and `behavior_server.yaml` (speeds, accelerations) | Dev 5 platform | Conservative placeholders. Per robot: `config/robots/<robot>/nav2_limits.yaml`, selected with `robot:=<robot>` |
| `FollowPath.inflation_cost_scaling_factor` (3.0) | Dev 3 | Must equal the local inflation layer's `cost_scaling_factor` |
| Global inflation layer `inflation_radius` | Dev 3 | Smac2D needs it to be **at least half the robot's largest cross-section**, otherwise its collision checking degrades and it logs *"inflation is not set sufficiently"* |
| Inflation layer `inflate_around_unknown: true` (both costmaps) | Dev 3 | Architecture §8.1 "unknown = never free — inflate". Without it paths hug unknown space; in the closed-loop `unknown_block` run the robot cut a corner, ended with its centre in unknown cells and Smac2D (`allow_unknown: false`) aborted with *"Start occupied"*. With it, the robot keeps ≥ 0.5 m from unknown |
| Feeding Dev 3's grid to Nav2 | Dev 3 | Proposal, proven in the closed-loop fixture: publish an `OccupancyGrid` (0 free, 100 lethal, -1 unknown, 1..99 scaled) and read it in both costmaps with `nav2_costmap_2d::StaticLayer` (`map_topic`, `track_unknown_space: true`) + inflation. Works with the rolling `odom` local costmap. Nav2 itself publishes `/global_costmap/costmap` and `/local_costmap/costmap`, so Dev 3 needs a different topic name |
| Costmap `update_frequency` / `publish_frequency` | Dev 3 | Sets how fast hazards reach Smac2D/RPP. Local costmap ≥ 5 Hz is recommended |

## External prerequisites (not launched here)

- **Dev 3:** costmap params file with `global_costmap` and `local_costmap` sections
  (layers, footprint/robot_radius, inflation). The launch file uses Dev 3's
  `config/costmaps.yaml` in this package by default when it exists, or
  `costmap_params_file:=<path>`. With neither it refuses to start.
- **Dev 2:** TF `map → odom → base_link` and `/odom`. Nav2 activation blocks until TF exists.
- **Dev 5:** the safety authority consuming `/cmd_vel_nav2` and owning `/cmd_vel`,
  plus the robot/sim.

## Commands

Without a native Lyrical install, build + run every test in Docker (from the repo root;
nothing is written back to the repo):

```bash
src/ugv_navigation/docker/test_in_lyrical.sh src/ugv_navigation
```

Native Lyrical:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select ugv_navigation
colcon test --packages-select ugv_navigation && colcon test-result --verbose
source install/setup.bash

# Launch (uses Dev 3's config/costmaps.yaml if present; otherwise pass the file)
ros2 launch ugv_navigation navigation.launch.py
ros2 launch ugv_navigation navigation.launch.py costmap_params_file:=/path/to/dev3_costmaps.yaml

# Separate processes instead of one container (debugging)
ros2 launch ugv_navigation navigation.launch.py use_composition:=false

# Per robot: limits (config/robots/<robot>/nav2_limits.yaml) + Dev 5's
# config/robots/footprint_<robot>.yaml if found above this package
ros2 launch ugv_navigation navigation.launch.py robot:=secondary
ros2 launch ugv_navigation navigation.launch.py robot:=primary footprint_file:=/path/to/footprint.yaml

# Nav2 liveness for Dev 5's watchdog (false = hold) and the reason
ros2 topic echo /ugv/nav2_heartbeat
ros2 topic echo /ugv/nav2_status

# Boundary / action / lifecycle check
ros2 run ugv_navigation nav_goal_testbench check

# A->B goals (map frame, x,y[,yaw]) + benchmark table
ros2 run ugv_navigation nav_goal_testbench goal 3.0,0.0 3.0,2.0,1.57 --csv runs.csv

# Closed loop without Dev 2/3/5 (TEST-ONLY fake base + synthetic map, from the repo root):
#   open | wall_gap | corridor | unknown_block | dynamic_obstacle
ros2 launch src/ugv_navigation/test/closed_loop/closed_loop.launch.py scenario:=wall_gap \
  footprint_file:=src/ugv_navigation/test/fixtures/test_only_footprint_primary.yaml
ros2 run ugv_navigation nav_goal_testbench goal 4.0,0.0

# Raw CLI equivalents
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}}" --feedback
ros2 topic echo /cmd_vel_nav2
ros2 topic info /cmd_vel --verbose   # no Dev 4 node may appear as a publisher
```
