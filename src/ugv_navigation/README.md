# ugv_navigation — Dev 4: Planning & Trajectory Control

Nav2 planning/control core (architecture.md §3.1, §6, §11; dev.md Dev 4).
The costmap subsystem in this package belongs to Dev 3 and is **not** configured here.

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
| `config/controller_server.yaml` | 2 RPP (adaptive lookahead, curvature + cost regulation, collision detection, rotate-to-heading) · 3 fresh-costmap wait · 5 unstamped `Twist` |
| `behavior_trees/navigate_to_pose_ugv.xml` | 3 replan at 2 Hz if path > 2 s old / goal updated / `IsPathValid` fails · 4 spin / wait / backup |
| `config/behavior_server.yaml` | 4 recovery plugins (spin, backup, wait) |
| `config/bt_navigator.yaml` | 4 `/navigate_to_pose` |
| `launch/navigation.launch.py` | all; 5 remaps every Nav2 `cmd_vel` to `/cmd_vel_nav2` |
| `scripts/nav_goal_testbench`, `ugv_navigation/testbench_core.py` | 6 CLI / goal benchmark |
| `test/` | static contract tests, testbench unit tests, live graph boundary test |

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

## Values to confirm with other devs

| Key | Owner | Why |
|---|---|---|
| `[ROBOT LIMIT]` values in `controller_server.yaml` and `behavior_server.yaml` (speeds, accelerations) | Dev 5 platform | Conservative placeholders. Override per robot with `robot_params_file:=config/robots/<robot>/<file>.yaml` |
| `FollowPath.inflation_cost_scaling_factor` (3.0) | Dev 3 | Must equal the local inflation layer's `cost_scaling_factor` |
| Global inflation layer `inflation_radius` | Dev 3 | Smac2D needs it to be **at least half the robot's largest cross-section**, otherwise its collision checking degrades and it logs *"inflation is not set sufficiently"* |
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

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ugv_navigation
colcon test --packages-select ugv_navigation && colcon test-result --verbose
source install/setup.bash

# Launch (uses Dev 3's config/costmaps.yaml if present; otherwise pass the file)
ros2 launch ugv_navigation navigation.launch.py
ros2 launch ugv_navigation navigation.launch.py costmap_params_file:=/path/to/dev3_costmaps.yaml

# Boundary / action / lifecycle check
ros2 run ugv_navigation nav_goal_testbench check

# A->B goals (map frame, x,y[,yaw]) + benchmark table
ros2 run ugv_navigation nav_goal_testbench goal 3.0,0.0 3.0,2.0,1.57 --csv runs.csv

# Raw CLI equivalents
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}}" --feedback
ros2 topic echo /cmd_vel_nav2
ros2 topic info /cmd_vel --verbose   # no Dev 4 node may appear as a publisher
```
