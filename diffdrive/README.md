# diffdrive workspace

A differential-drive robot in Gazebo with a simulated Livox Mid-360 (`diff_description`),
MOLA lidar SLAM/localization (`mola_bringup`, `MOLA-SLAM`), and Nav2 autonomous navigation
on top of MOLA's localization.

## One-time setup

All launch files here pin `ROS_DOMAIN_ID=161` to avoid DDS cross-talk with other ROS 2
machines on the same LAN (this workspace was built on a shared network where other
machines default to domain 0). For manual commands in other terminals (teleop, `ros2
topic echo`, etc.) run:

```bash
export ROS_DOMAIN_ID=161
```

This is already added to `~/.bashrc`, so new terminals pick it up automatically.

## 1. Drive the robot in Gazebo

```bash
ros2 launch diff_description rviz_joint_control.launch.py
```

Starts Gazebo + the robot + RViz. Drive it with, in another terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 2. Build a map with MOLA SLAM

```bash
ros2 launch mola_bringup mola_slam_launch.py
```

Drive the robot around (teleop) to cover the space, then save the map:

```bash
ros2 service call /map_save mola_msgs/srv/MapSave "map_path: '/home/andy1/ws_ros2_test/maps/<name>'"
```

This produces `<name>.mm` / `<name>.simplemap` - MOLA's own 3D metric map format.

## 3. Localize against a saved map

```bash
ros2 launch mola_bringup mola_localize_launch.py
```

Edit the `mm_map`/`simple_map` paths at the top of that launch file to point at the map
you want to localize against (defaults to `myroom.mm`/`myroom.simplemap`).

MOLA starts paused (`active: false`). Activate it with:

```bash
ros2 service call /mola_runtime_param_set mola_msgs/srv/MolaRuntimeParamSet \
  "{parameters: 'mola::LidarOdometry:lidar_odom:\n  active: true\n'}"
```

(The `plot_lidar_trajectory.py` GUI launched alongside this does the same thing via its
"Publish Initial Pose & Activate MOLA" button, plus lets you set a non-origin initial
pose if you know roughly where the robot starts.)

**If localization stops updating (position frozen while the robot keeps moving in
Gazebo):** driving too fast/aggressively (e.g. sustained combined forward+turn) can make
MOLA's ICP lose tracking - `icp_quality` in `/mola_diagnostics/lidar_odom/status` drops
towards 0 when this happens. Recover it without restarting the whole stack:

```bash
ros2 service call /relocalize_near_pose mola_msgs/srv/RelocalizeNearPose \
  "{pose: {header: {frame_id: 'map'}, pose: {pose: {position: {x: <x>, y: <y>, z: 0.0}, orientation: {w: 1.0}}, covariance: [4,0,0,0,0,0, 0,4,0,0,0,0, 0,0,4,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1]}}}"
```

using your best estimate of the current position. Driving more gently (lower linear/
angular speed, less continuous spinning) avoids triggering this in the first place. On
long sessions MOLA's own internal processing can also gradually slow down independent of
this - if `average_process_time` in the diagnostics topic keeps climbing over many
minutes, that's MOLA's own pipeline cost growing (not something this workspace's code
controls), and a relocalize/restart is the practical workaround for now.

## 4. Autonomous navigation with Nav2

Nav2 is **not** installed by default here - one-time setup:

```bash
sudo apt-get install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

MOLA (not AMCL) provides `map -> odom` localization, so Nav2 here only handles costmaps,
planning and control. Its costmap obstacle layers read the live 3D point cloud directly
(`VoxelLayer`), not a flattened 2D scan.

### 4a. Build a 2D occupancy grid for Nav2's static map layer

MOLA's own map (`.mm`/`.simplemap`) is a 3D point-cloud/pose-graph format and can't be
loaded by Nav2's `map_server` directly. Build a 2D grid instead by driving around with
localization active (step 3) and this extra node running:

```bash
ros2 launch mola_bringup build_2d_map_launch.py
```

It height-slices the live filtered point cloud (walls/furniture, not floor/ceiling),
projects it into a 2D grid in the `map` frame, and publishes it on `/map`. Drive around
gently (see the tracking-loss note above) to cover the area, then save it with Nav2's own
tool:

```bash
ros2 run nav2_map_server map_saver_cli -f /home/andy1/ws_ros2_test/maps/<name>_2d
```

Note this map only marks cells it actually got lidar returns on as occupied; everywhere
else stays "unknown" rather than being cleared to "free" (no ray-tracing pass). That's
fine here - the live point-cloud costmap layer (below) handles real-time obstacle
avoidance regardless of what the static layer knows, so the static layer is only there to
help the global planner route around walls it's already seen. `allow_unknown: true` is
set on the planner and `track_unknown_space: true` on both costmaps to match.

### 4b. Run Nav2

With Gazebo + `mola_localize_launch.py` (activated) already running:

```bash
ros2 launch mola_bringup nav2_bringup_launch.py map:=/home/andy1/ws_ros2_test/maps/<name>_2d.yaml
```

Send a goal either through RViz's "2D Nav Goal" tool, or directly:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
```

Config lives in `mola_bringup/config/nav2_params.yaml`. The robot footprint is
approximated as a 0.4 m radius circle (its actual footprint is a 0.725 x 0.3 m
rectangle) - fine for open-space navigation, but tune this (or switch to an explicit
`footprint` polygon) if it needs to fit through tight gaps.
