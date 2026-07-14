# diffdrive workspace

A differential-drive robot in Gazebo with a simulated Ouster OS1 lidar +
industrial-grade IMU + GPS/GNSS receiver, LIO-SAM SLAM, and Nav2 autonomous
navigation - indoors, outdoors (GPS-assisted), at campus scale (satellite-map
goal picking + graph-constrained routing), and with a full data-collection ->
SLAM -> georeferenced-GIS-map workflow.

## Prerequisites

Targets **ROS 2 Jazzy on Ubuntu 24.04**. Install ROS 2 Jazzy itself first (see
https://docs.ros.org/en/jazzy/Installation.html), then:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-xacro ros-jazzy-urdf-tutorial \
  ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-gz-ros2-control \
  ros-jazzy-controller-manager ros-jazzy-diff-drive-controller ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-robot-state-publisher ros-jazzy-rviz2 \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-pcl-ros ros-jazzy-pcl-conversions libeigen3-dev python3-dev \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-gtsam \
  ros-jazzy-robot-localization ros-jazzy-nav2-route ros-jazzy-rviz-satellite \
  proj-bin python3-pil python3-yaml \
  python3-tk python3-matplotlib python3-numpy
```

**The repo root (`~/ws_ros2_test/src`, the folder with `diffdrive/` directly
inside it) doubles as colcon's build root** - `build/`/`install`/`log` land
inside `~/ws_ros2_test/src/` itself, not one level up. From
`~/ws_ros2_test/src`:

```bash
sudo rosdep init   # only if you've never run rosdep on this machine before
rosdep update
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Add `source ~/ws_ros2_test/src/install/setup.bash` to `~/.bashrc` so new
terminals pick up the built packages automatically.

## One-time setup

All launch files pin `ROS_DOMAIN_ID=161` (avoids DDS cross-talk on a shared
LAN). For manual commands in other terminals:

```bash
export ROS_DOMAIN_ID=161
```

Already in `~/.bashrc`, so new terminals pick it up automatically.

## Indoor

### Drive manually

```bash
ros2 launch diff_description rviz_joint_control.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### SLAM

```bash
ros2 launch lio_sam_bringup lio_sam_slam_launch.py
```

SLAM nodes start ~20s in (robot needs to settle from its physics-drop spawn
first). If `imuPreintegration` isn't in `ros2 node list` after that, just
relaunch.

### Save the map

While SLAM is still running:

```bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
  "{resolution: 0.2, destination: '/ws_ros2_test/maps/lio_sam_house'}"
```

`destination` is appended directly after `$HOME` - give it as the part
*after* `$HOME` (leading `/`, no `$HOME` prefix) as above, or it silently
saves into `~/home/andy/...`.

### Localize on a saved map

```bash
ros2 launch lio_sam_bringup lio_sam_localization_launch.py \
  map_pcd_path:=/home/andy/ws_ros2_test/maps/lio_sam_house/GlobalMap.pcd \
  initial_x:=0.0 initial_y:=0.0 initial_z:=0.0 initial_yaw:=0.0
```

`initial_x/y/z/yaw` seed the ICP search - a fresh run's `odom` frame starts
wherever the robot spawns *this* time. Get this wrong and ICP converges to
the wrong place (or not at all); nudge live via RViz's "2D Pose Estimate"
tool.

### Nav2

```bash
# Alongside SLAM (no prior map, live-built rolling costmap):
ros2 launch lio_sam_bringup nav2_bringup_launch.py

# Alongside localization on a saved map (static map + live obstacles):
ros2 launch lio_sam_bringup nav2_bringup_launch.py \
  params_file:=/home/andy/ws_ros2_test/src/install/lio_sam_bringup/share/lio_sam_bringup/config/nav2_params_localization.yaml
```

Send a goal via RViz's "2D Nav Goal" or directly:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
```

## Outdoor

### GPS-assisted SLAM + Nav2

```bash
ros2 launch lio_sam_bringup lio_sam_gps_outdoor_launch.py
# once it's settled (~90s):
ros2 launch lio_sam_bringup nav2_bringup_launch.py \
  params_file:=/home/andy/ws_ros2_test/src/install/lio_sam_bringup/share/lio_sam_bringup/config/nav2_params_outdoor.yaml
```

### Campus-scale routing (satellite map + graph-constrained paths)

```bash
ros2 launch lio_sam_bringup lio_sam_gps_outdoor_launch.py
ros2 launch lio_sam_bringup nav2_bringup_launch.py \
  params_file:=/home/andy/ws_ros2_test/src/install/lio_sam_bringup/share/lio_sam_bringup/config/nav2_params_outdoor_route.yaml
```

RViz opens with a real satellite basemap under the "2D Nav Goal" tool -
click a location to send a goal, same as before.

To build a real campus route graph from surveyed lat/lon waypoints (run
against a live outdoor launch):

```bash
ros2 run lio_sam_bringup latlon_graph_builder.py \
  --input campus_waypoints.yaml --output campus_graph.geojson
```

Then pass `graph_filepath:=/path/to/campus_graph.geojson` on the Nav2
launch command above.

### Campus mapping workflow (data collection -> SLAM -> georeferenced GIS map)

1. **Collect data**: launch outdoor GPS mode above and drive the robot
   around the target area.
2. **Save the map**: same as indoor -
   `ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution:
   0.2, destination: '/path/to/campus_map'}"`.
3. **Smooth position tracking**: already running automatically as part of
   `lio_sam_gps_outdoor_launch.py` - no extra command. Publishes
   `/monitoring/gps_fix` (smoothed lat/lon) for external GIS/dashboard
   tools.
4. **Georeference the saved map** (with `pcd_map_localizer` running in
   localization mode against the campus map, and a live outdoor launch up
   for `/toLL`):

```bash
ros2 run nav2_map_server map_saver_cli -f campus_map
ros2 run lio_sam_bringup georeference_map.py --map campus_map.yaml
```

Load `campus_map.pgm` directly into QGIS and assign the printed EPSG code
to see it correctly positioned against real-world imagery/GIS layers.
