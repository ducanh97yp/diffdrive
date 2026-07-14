# diffdrive workspace

A differential-drive robot in Gazebo with a simulated Ouster OS1 lidar +
industrial-grade IMU (`diff_description`), LIO-SAM tightly-coupled lidar-inertial
SLAM (`lio_sam`, `lio_sam_bringup`), and Nav2 autonomous navigation on top of it.

LIO-SAM + OS1 replaced an earlier FAST-LIO + Livox Mid-360 pipeline at the
customer's explicit request (GTSAM factor-graph/loop-closure architecture,
Ouster's mechanical 360°-native scan pattern), not because FAST-LIO was
deficient.

## Test results (Gazebo ground truth vs SLAM's own odometry)

| Scenario | LIO-SAM (OS1) |
|---|---|
| Basic odometry (straight line) | 0.036% (1.7mm over 4.8m) |
| Wheel slip (low-friction patch) | no direct effect - LIO-SAM uses no wheel odometry |
| Sustained wall collision | yaw error ~0.01°, position error 0.43% |
| 8° ramp (`terrain_test.world`) | not clean - see [known limitation](#known-limitation-terrain_testworld-and-open-terrain) |

Basic odometry, wheel-slip, and wall-collision are all solid. The ramp scenario
surfaced a real limitation specific to LIO-SAM's feature-based front end,
documented below.

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
  python3-tk python3-matplotlib python3-numpy
```

`ros-jazzy-gtsam` is a plain apt package on Jazzy - no from-source build needed.

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

## 1. Drive the robot in Gazebo

```bash
ros2 launch diff_description rviz_joint_control.launch.py
```

Starts Gazebo + the robot + RViz. Drive it with:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 2. Run SLAM with LIO-SAM

```bash
ros2 launch lio_sam_bringup lio_sam_slam_launch.py
```

Starts Gazebo + `gz_lidar_to_ouster` (adapts Gazebo's simulated lidar cloud to
LIO-SAM's `OusterPointXYZIRT` layout) + LIO-SAM's 4 nodes + a static identity
`map -> odom` link (LIO-SAM folds pose-graph corrections into what it
publishes as `odom` directly) + RViz.

SLAM nodes start **20 seconds in** - starting immediately, while the robot is
still settling from its physics-drop spawn, causes a
`gtsam::IndeterminantLinearSystemException` crash in `imuPreintegration`. This
crash can still occasionally happen even with the delay (non-deterministic);
if `nav2_bringup_launch.py` never activates or `imuPreintegration` isn't in
`ros2 node list`, just relaunch.

Position is published on `/lio_sam/mapping/odometry` (frame `odom`), not
`/diff_cont/odom` - LIO-SAM uses no wheel odometry.

## 3. Save the map and localize on a saved map

LIO-SAM has no localization-only mode - it only ever runs live SLAM.
`lio_sam_bringup` adds a small ICP-based node (`pcd_map_localizer`) to
relocalize against a previously saved map.

### 3a. Save the map

While step 2 is still running:

```bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
  "{resolution: 0.2, destination: '/ws_ros2_test/maps/lio_sam_house'}"
```

**`destination` is appended directly after `$HOME`, not treated as an
absolute path** (upstream LIO-SAM behavior: `std::getenv("HOME") +
req->destination`, no separator inserted). Give `destination` as the part
*after* `$HOME` (leading `/`, no `$HOME` prefix) as above, or it silently
saves into `~/home/andy/...`. Writes `GlobalMap.pcd` (what step 3b loads),
`CornerMap.pcd`, `SurfMap.pcd`, `trajectory.pcd`, `transformations.pcd`.

### 3b. Localize on the saved map

```bash
ros2 launch lio_sam_bringup lio_sam_localization_launch.py \
  map_pcd_path:=/home/andy/ws_ros2_test/maps/lio_sam_house/GlobalMap.pcd \
  initial_x:=0.0 initial_y:=0.0 initial_z:=0.0 initial_yaw:=0.0
```

Identical to `lio_sam_slam_launch.py` except the static `map -> odom` link is
replaced by `pcd_map_localizer`: it ICP-registers the current scan against
the loaded `GlobalMap.pcd` every `icp_update_period` (default 0.2s) and
broadcasts the resulting `map -> odom` correction continuously at 50Hz.

`initial_x/y/z/yaw` seed the ICP search - a fresh run's `odom` frame starts
wherever the robot spawns *this* time, rarely where mapping started. Get this
wrong and ICP converges to the wrong place (or not at all); nudge live via
RViz's "2D Pose Estimate" tool (`/initialpose`, same mechanism AMCL uses).

RViz shows the loaded map on `/prior_map` (latched PointCloud2, full
resolution). `pcd_map_localizer` also projects the map down to a 2D
`nav_msgs/OccupancyGrid` on `/map` (same topic/QoS pattern as
`nav2_map_server`), so Nav2's `static_layer` and RViz's Map display pick it up
with no extra bridging node - a simple height-slice projection
(`map2d_z_min`/`map2d_z_max`, default 0.05-1.5m), not survey-grade.

**Degeneracy-aware ICP correction**: plain point-to-point ICP can converge
confidently (low fitness score) to a *wrong* alignment when the local
geometry only constrains some directions - e.g. a corridor of parallel walls
constrains y/z but not x, so sliding along x barely changes the fit.
`applyDegeneracyAwareCorrection()` in `pcd_map_localizer.cpp` builds a
translation information matrix from the surface normals at each
correspondence, eigen-decomposes it, and only accepts the ICP correction
along directions that are actually well-constrained
(`icp_degenerate_eigenvalue_ratio`, default 0.45) - rejected directions keep
the previous estimate instead of trusting a direction the scan couldn't see.
A second safeguard (`icp_max_correction_per_update`, default 0.3m) caps how
far a single update may move `map -> odom`, since even a merely-weak (not
fully singular) axis can occasionally produce a large single-frame jump. This
correctly means the node sometimes *declines* to correct drift on genuinely
under-constrained terrain (e.g. featureless flat ground/ramps) rather than
confidently reporting a wrong position - an honest limit, not a bug.

Tuning knobs (both node params and `lio_sam_localization_launch.py` launch
args): `icp_update_period`, `icp_max_correspondence_distance`,
`icp_fitness_score_threshold`, `icp_voxel_leaf_scan`,
`icp_degenerate_eigenvalue_ratio`, `icp_max_correction_per_update`,
`tf_tolerance` (default 0.2s - `map -> odom` is stamped this far into the
future, the same trick AMCL uses, so a TF lookup that lands between two
broadcasts doesn't throw an extrapolation error and silently stall Nav2).

## 4. Autonomous navigation with Nav2

```bash
# Alongside step 2 (mapping) - no prior map, live-built rolling costmap:
ros2 launch lio_sam_bringup nav2_bringup_launch.py

# Alongside step 3 (localizing on a saved map) - static map + live obstacles:
ros2 launch lio_sam_bringup nav2_bringup_launch.py \
  params_file:=/home/andy/ws_ros2_test/install/lio_sam_bringup/share/lio_sam_bringup/config/nav2_params_localization.yaml
```

LIO-SAM (not AMCL) provides localization directly - `map -> odom` (static or
ICP-corrected, see step 3b) `-> base_link` (dynamic, from
`imuPreintegration`/`TransformFusion`). No custom TF bridge is needed since
LIO-SAM's TF tree already matches the URDF.

The two params files differ in `global_costmap`: `nav2_params.yaml` (mapping
mode) has no `static_layer` (nothing to load yet) and stays `rolling_window`;
`nav2_params_localization.yaml` adds a `static_layer` reading `/map` (from
step 3b) and auto-sizes to match it, the standard Nav2 pattern for a
known-map robot. Send a goal via RViz's "2D Nav Goal" or directly:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
```

Config lives in `nav2_params.yaml`/`nav2_params_localization.yaml` (kept
separate rather than shared, since a `static_layer` present but never fed
`/map` during mapping mode would leave the whole costmap stuck "not
current"). Robot footprint is a 0.2m radius circle.

### Driving speed

`FollowPath`'s MPPI params were tuned for faster driving: `vx_max`/`vx_min`
0.5/-0.35 -> 1.0/-0.7 m/s, `wz_max` 1.9 -> 2.5 rad/s. Raising the velocity
ceiling alone doesn't make the robot actually drive faster - MPPI samples
control noise around the *current* velocity, so `vx_std` was also raised
(0.2 -> 0.8) and `PathFollowCritic.offset_from_furthest` (the "carrot" point
MPPI chases progress toward) raised from 5 to 25 path points (~0.25m -> ~1.25m
ahead) - without a farther carrot there's no reward for going faster to reach
it.

### Climbing ramps: `ElevationTraversabilityLayer`

Stock Nav2 (`ObstacleLayer`/`VoxelLayer`) marks *any* lidar point with `z >=
0` as an obstacle - no minimum height, so a gentle ramp gets treated exactly
like a wall and Nav2 refuses to climb it (even though manual teleop, which
never consults the costmap, climbs fine). Fixed with a custom
`nav2_costmap_2d::Layer` plugin,
`lio_sam_bringup::ElevationTraversabilityLayer`
(`src/elevation_traversability_layer.cpp` +
`include/lio_sam_bringup/elevation_traversability_layer.hpp`, registered via
`costmap_plugins.xml`), that marks cells lethal by **local slope** instead of
absolute height. It **replaces** `obstacle_layer`/`voxel_layer` in both yaml
files (all four costmap sections) - coexisting wouldn't work, since
`CostmapLayer::updateWithMax`'s max-combination would let the old layer's
lethal marks win regardless of plugin order.

How it works: tracks a per-cell height estimate from the live point cloud
(instantly adopts a *lower* reading - the true surface is the lowest
consistent return - recovers slowly from a higher one so a single outlier
can't poison it), then computes the steepest slope to neighbors 4 cells away
(0.2m baseline at 0.05m resolution) and thresholds: free below 15°, lethal
at/above 35°, scaled cost between. Publishes a
`<costmap>/elevation_traversability_layer/debug_slope_grid`
(`nav_msgs/OccupancyGrid` of slope-in-degrees) for inspecting *why* a region
reads the way it does in RViz. Full parameter list is in the plugin's own
header/source comments.

Real walls/furniture aren't lost - they still read near-90° local slope
regardless of exact height. What's genuinely given up vs. the old raytracing
layers is dynamic-obstacle *clearing*; substituted by resetting the rolling
local costmap's grid on every window shift and a slow-recovery filter for the
persistent global costmap. `obstacle_min_range` defaults to 0.6m (not 0.0) -
lower than that, a self-hit point leaking past `gz_lidar_to_ouster`'s own
self-filter box gets permanently adopted into the height grid with no
clearing mechanism to ever remove it for a stationary robot.

**Tested end-to-end**: a Nav2 goal past `house.world`'s ramp **succeeded**
(`status: SUCCEEDED`) under full MPPI control, pitch tracking the ramp's known
6° geometry through the climb. Negative control: a goal past the world's real
far wall still correctly stalled at the wall.

## Package layout

- **`diff_description`** - robot URDF/xacro (Ouster OS1 lidar + industrial
  IMU), Gazebo worlds, ros2_control config, base Gazebo bringup launch file.
- **`lio_sam_bringup`** - the Gazebo/Nav2 integration: `gz_lidar_to_ouster`
  point cloud adapter, `pcd_map_localizer` ICP relocalization node,
  `ElevationTraversabilityLayer` Nav2 costmap plugin, SLAM/localization/Nav2
  launch files + configs.
- **`lio_sam`** - vendored `TixiaoShan/LIO-SAM` `ros2` branch, patched for
  Jazzy: `find_package(Eigen REQUIRED)` -> `find_package(Eigen3 REQUIRED)`,
  plus routing `imuPreintegration`'s Eigen include through
  `target_include_directories` directly.
- **`GUI`** - a small teleop GUI (`teleop_gui.py`).

### Note on simulation vs. real hardware

Gazebo's lidar sensor is a generic `gpu_lidar`, not a real OS1 - it
approximates the FOV shape and channel/column counts (N_SCAN=32,
Horizon_SCAN=512, real OS1-32 values) but not the actual rotating-mirror scan
mechanism. `gz_lidar_to_ouster.cpp` builds LIO-SAM's exact
`OusterPointXYZIRT` layout from the raw simulated cloud.

`ouster-ros` itself is not vendored or needed for simulation - LIO-SAM parses
the Ouster point layout by field name, not via the vendor driver. Real
deployment would need: `ouster-ros` (real per-point timestamps - our `t` is
always 0, correct only for Gazebo's instantaneous full-sweep simulation),
physical extrinsic calibration (`extrinsicTrans`/`extrinsicRot`/`extrinsicRPY`
in `lio_sam_params.yaml` come from xacro joint origins here, not measurement),
and lidar-IMU hardware time sync.

### Note on `extrinsicRot`/`extrinsicRPY`

LIO-SAM's `imuConverter()` applies `extrinsicRot` to raw accel/gyro and
`extrinsicRPY` to orientation, rotating from the IMU's frame into the lidar's
frame - **this is the physical lidar-IMU mounting rotation**, not an internal
IMU-chip axis quirk (FAST-LIO's analogous `extrinsic_R` has different
semantics - only rotates raw vectors, never treats orientation as a pose to
fuse). Both are identity here because the OS1 mount really is unrotated (its
FOV is symmetric ±22.5°, unlike the old Mid-360 mount which needed a
180°-about-X flip for its asymmetric FOV) - not because of a misreading of
`imuConverter()`. Getting this wrong (identity when a real rotation exists,
or a large rotation that pushes `mapOptmization`'s roll/pitch fusion into a
Euler-angle singularity) causes IMU divergence or catastrophic pose jumps.
**If a future lidar swap needs a downward FOV bias, encode it in the
`gpu_lidar` sensor's native vertical `min_angle`/`max_angle`**, not a physical
mount rotation, to avoid this class of bug.

### Note on lidar self-occlusion (OS1 mount)

Self-hit cluster at range 0.24-0.50m, z banded to -0.117 to -0.078m in the
lidar's own frame (grazing the chassis's flat top). Filtered via an
axis-aligned self-filter box in `gz_lidar_to_ouster.cpp`'s `self_filter_*`
parameters. If you change the lidar mount position/height or chassis
dimensions, recompute both the mount clearance and these parameters.

### Known limitation: `terrain_test.world` and open terrain

LIO-SAM's front end (LOAM-style discrete edge/surface features) needs
geometrically distinguishing, non-coplanar structure nearby to constrain
x/y/yaw well - unlike FAST-LIO's continuous ikd-tree matching, which
tolerated the same sparse `terrain_test.world` spawn area fine. Confirmed
empirically: LIO-SAM's pose drifted noticeably at rest despite normal feature
counts - too many *coplanar* features (open flat ground), not too few
overall. Mitigated by adding small 3D geometry near spawn (`spawn_wall_*`,
`spawn_box_*`, `spawn_anchor_wall`), reducing at-rest drift from an unbounded
~30m runaway to a bounded amount, but **not fully eliminating drift during
the ramp climb itself** (~26% position error in the last measured run).
`house.world` has no equivalent issue - furniture-dense enough that this
degeneracy never surfaces.

Follow-up options, in rough order of effort: more permanent geometry along
the ramp run-up itself (not just near spawn), tune
`edgeThreshold`/`surfThreshold` to prefer discriminating features, or
re-enable loop closure (currently disabled in `lio_sam_params.yaml` as a
precaution, never confirmed as cause or irrelevant).

The same parallel-wall geometry near spawn also causes an analogous
degeneracy for `pcd_map_localizer`'s ICP correction (see step 3b) - that
specific spurious-lock bug is fixed there, but the fix is honest rather than
magic: `pcd_map_localizer` correctly *declines* to correct position on this
world's mostly-flat ramp/platform (genuinely under-constrained, not a bug)
rather than silently trusting LIO-SAM's own drift. Both issues share the same
fix direction: more permanent 3D structure along the ramp/platform.

### `house.world`'s small ramp + ridges

`house.world` also has a small up/flat/down ramp course with two low ridges
("gờ") for repeatable stability testing without switching worlds - gentler
than `terrain_test.world`'s ramp: 6°, 1m run per side, ~10.5cm peak, two 2cm
ridges on the flat top. Placed at `x=5.3` to `x~7.989`, `y` within `±1.25`,
clear of both the house's furniture and the `slippery_patch` fixture (ends at
`x=5`). Goes up, across both ridges, and back down to ground level - a
repeatable cycle rather than a one-off drop-off.
