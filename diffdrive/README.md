# diffdrive workspace

A differential-drive robot in Gazebo with a simulated Ouster OS1 lidar +
industrial-grade IMU (`diff_description`), LIO-SAM tightly-coupled lidar-inertial
SLAM (`lio_sam`, `lio_sam_bringup`), and Nav2 autonomous navigation on top of it.

LIO-SAM + OS1 replaced an earlier FAST-LIO + Livox Mid-360 pipeline (which itself
replaced an even earlier MOLA-based one) at the customer's direction, wanting
LIO-SAM's GTSAM factor-graph/loop-closure architecture and Ouster's mechanical
360°-native scan pattern over FAST-LIO's single growing ikd-tree and Livox's
non-repetitive scan. Migrating an already-working, validated stack is a real cost,
and this repo's history includes swaps that were prototyped and reverted (RoboSense
Helios 5515 back to Mid-360) - this migration went forward because it was a direct,
explicit customer request, not because FAST-LIO was deficient.

## Test results (Gazebo ground truth vs SLAM's own odometry, `gz model -m diff_robot -p`)

| Scenario | LIO-SAM (OS1) | FAST-LIO (Mid-360, previous stack) |
|---|---|---|
| Basic odometry (straight line) | 0.036% (1.7mm over 4.8m) | <0.1-5.5%, varies by test |
| Wheel slip (low-friction patch) | same run as above - LIO-SAM uses no wheel odometry, so slip has no direct effect, same as FAST-LIO | ~1.4% aggregate |
| Sustained wall collision | yaw error ~0.01° (0.2180 rad vs ground truth 0.2181 rad), position error 0.43% | yaw < 0.2° |
| 8° ramp (`terrain_test.world`) | **not clean - see limitation below** | pitch error 0.85° |

Basic odometry, wheel-slip, and wall-collision are all excellent - as good as or
better than the FAST-LIO baseline. The ramp/`terrain_test.world` scenario surfaced a
real, understood limitation specific to LIO-SAM's architecture, documented in
[its own section below](#known-limitation-terrain_testworld-and-open-terrain).

## Prerequisites

Targets **ROS 2 Jazzy on Ubuntu 24.04**. Install ROS 2 Jazzy itself first (Desktop or
Base - see https://docs.ros.org/en/jazzy/Installation.html), then install every
package this workspace needs at build/run time:

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

`ros-jazzy-gtsam` (LIO-SAM's factor-graph backend) is an apt package on Jazzy - no
from-source build needed. This is a meaningful simplification over the previous
FAST-LIO stack, which needed a separately-built, patched `Livox-SDK2` +
`livox_ros_driver2` as a hard build dependency regardless of which lidar was actually
in use; LIO-SAM has no such dependency (it parses generic `sensor_msgs/PointCloud2`
by field name, not a vendor SDK).

**The repo root (`~/ws_ros2_test/src`, the folder with `diffdrive/` directly inside
it) doubles as colcon's build root here** - there's no extra `src/` nesting beneath
it, so you build directly from there, and `build/`/`install`/`log` land inside
`~/ws_ros2_test/src/` itself (**not** `~/ws_ros2_test/`, one level up - that's an
unrelated, stale leftover from earlier in this project's history if it exists on your
machine; delete it if so, or you'll get confusing "package not found" errors from
whichever one your shell happens to source). From `~/ws_ros2_test/src`:

```bash
sudo rosdep init   # only if you've never run rosdep on this machine before
rosdep update
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Add `source ~/ws_ros2_test/src/install/setup.bash` (that exact path - **not**
`~/ws_ros2_test/install/setup.bash`, a different, unrelated location) to `~/.bashrc`
so new terminals pick up the built packages automatically (alongside `ROS_DOMAIN_ID`
below).

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

## 2. Run SLAM with LIO-SAM

```bash
ros2 launch lio_sam_bringup lio_sam_slam_launch.py
```

Starts Gazebo + the robot + `gz_lidar_to_ouster` (adapts Gazebo's raw simulated lidar
cloud to LIO-SAM's `OusterPointXYZIRT` layout) + LIO-SAM's 4 nodes
(`imageProjection`, `featureExtraction`, `imuPreintegration`, `mapOptimization`) + a
static identity `map -> odom` link (LIO-SAM folds pose-graph corrections directly
into what it publishes as `odom` - there's no separate corrected `map` frame, same
pattern LIO-SAM's own reference `run.launch.py` uses) + RViz.

Like FAST-LIO before it, everything SLAM-related only starts **20 seconds in** - not
because LIO-SAM's own init is slow, but because starting immediately (while the robot
is still settling from its physics-drop spawn) caused a `gtsam::
IndeterminantLinearSystemException` crash in `imuPreintegration` on first launch,
confirmed fixed by waiting for full settle first.

Position is published on `/lio_sam/mapping/odometry` (`nav_msgs/Odometry`, frame
`odom`), not `/diff_cont/odom` - LIO-SAM doesn't use wheel odometry at all (same
reason wheel slip doesn't affect it, matching FAST-LIO's behavior here).

**No map save/load or relocalize-against-a-saved-map mode** in this migration
(explicitly out of scope, matching this repo's precedent - FAST-LIO's own
`prior_map_path` relocalization was a separate follow-up request after its initial
migration, not part of the first pass). LIO-SAM's keyframe/pose-graph architecture
(GTSAM factors over `cloudKeyPoses3D/6D` + corner/surface submaps) makes this a
harder patch than FAST-LIO's single-ikd-tree `prior_map_path` was - LIO-SAM ships a
save-only `SaveMap.srv`, but loading a saved map to relocalize against would need a
bootstrapped prior keyframe/pose state or an ICP-based initial-pose seed, budgeted
separately if wanted later.

## 3. Autonomous navigation with Nav2

Nav2 (`ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`) is installed as part of
[Prerequisites](#prerequisites) above. Run this alongside step 2:

```bash
ros2 launch lio_sam_bringup nav2_bringup_launch.py
```

LIO-SAM (not AMCL) provides localization directly. The TF chain is `map -> odom`
(static identity, from step 2's launch file) `-> base_link` (dynamic, broadcast by
LIO-SAM's `imuPreintegration`/`TransformFusion`, which looks up the static
`laser_frame -> base_link` transform from the URDF itself to correct for the lidar
mount offset - **no custom TF bridge script is needed**, unlike FAST-LIO's
`fastlio_tf_bridge.py`, which existed only because FAST-LIO published
`camera_init -> body` with `body` outside the URDF's TF tree). `diff_cont`'s own
`odom -> base_link` broadcast stays disabled in
`diff_description/config/my_controllers.yaml` for the same reason as before - it
would otherwise fight LIO-SAM over who parents `base_link`.

Our simulated OS1 only publishes `PointCloud2` (no native 2D `LaserScan` bridge
exists), so both costmaps' obstacle/voxel layers observe `/ouster_points` directly as
a `PointCloud2` source. Global costmap uses `rolling_window: true` (without a
pre-built static map to load, a non-rolling costmap defaults to a small fixed box
anchored at world origin - confirmed via Nav2's own "out of map bounds" warning once
the robot drove outside it). Local costmap's voxel layer covers up to z=2.0m
(`z_voxels: 40`) - raised from an initial 0.8m after the same kind of "out of map
bounds" warning appeared with the robot elevated on `terrain_test.world`'s ramp
platform; `origin_z` is a fixed world-z, not robot-relative, so this ceiling is still
finite and won't follow the robot onto arbitrarily tall structures.

Send a goal either through RViz's "2D Nav Goal" tool (Fixed Frame `map`), or directly:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
```

Config lives in `lio_sam_bringup/config/nav2_params.yaml`. The robot footprint is
approximated as a 0.2m radius circle - fine for open-space navigation, but tune this
(or switch to an explicit `footprint` polygon) if it needs to fit through tight gaps.

## Package layout

- **`diff_description`** - robot URDF/xacro (Ouster OS1 lidar + industrial IMU),
  Gazebo worlds, ros2_control config, and the base Gazebo bringup launch file
  (`rviz_joint_control.launch.py`) everything else includes.
- **`lio_sam_bringup`** - the Gazebo/Nav2 integration built around LIO-SAM: the
  `gz_lidar_to_ouster` point cloud adapter node, and SLAM/Nav2 launch files + configs.
- **`lio_sam`** - vendored `TixiaoShan/LIO-SAM` `ros2` branch, one patch needed for
  Jazzy: `CMakeLists.txt`'s `find_package(Eigen REQUIRED)` -> `find_package(Eigen3
  REQUIRED)` (system provides `Eigen3Config.cmake`, not `EigenConfig.cmake`), plus
  routing the `imuPreintegration` target's Eigen include through
  `target_include_directories` directly instead of `ament_target_dependencies`.
- **`GUI`** - a small teleop GUI (`teleop_gui.py`).

### Note on simulation vs. real hardware

This project's Gazebo lidar sensor is a generic `gpu_lidar` (angular ray-cast grid)
regardless of which sensor the xacro describes - it can't reproduce OS1's real
rotating-mirror-per-channel scan mechanism, only approximate its FOV shape and
channel/column counts (N_SCAN=32, Horizon_SCAN=512 - real, common OS1-32 config
values, not arbitrary simulation-only numbers, so this stays meaningful when
transitioning to real hardware). Points always arrive as `PointCloud2` via
`ros_gz_bridge` and go through `lio_sam_bringup`'s adapter (`gz_lidar_to_ouster.cpp`),
which builds LIO-SAM's exact `OusterPointXYZIRT` layout - only `x/y/z/intensity/t/ring`
are actually consumed by LIO-SAM's `imageProjection.cpp` (confirmed by reading its
`SensorType::OUSTER` parsing branch directly), but `reflectivity`/`noise`/`range` are
still populated with physically-derived values (not left at zero) so a real
`ouster-ros` bring-up later sees a structurally familiar message.

**`ouster-ros` itself is not vendored or needed for simulation** - confirmed from
LIO-SAM's own `package.xml`/`CMakeLists.txt`, it only depends on generic
`sensor_msgs`/`pcl_conversions`/`GTSAM`, and parses the Ouster point layout by field
name rather than linking against the vendor driver. Real deployment would need:
`ouster-ros` itself (or an equivalent driver publishing real
`OusterPointXYZIRT`-compatible clouds with genuine per-point timestamps - our `t` is
always 0, correct for Gazebo's instantaneous full-sweep simulation but not for a real
rolling-shutter mechanical scan), physical extrinsic calibration (`extrinsicTrans`/
`extrinsicRot`/`extrinsicRPY` in `lio_sam_params.yaml` come from exact xacro joint
origins here, not measurement), and lidar-IMU hardware time sync.

### Note on `extrinsicRot`/`extrinsicRPY` - a real mistake made and fixed this session

LIO-SAM's `imuConverter()` (in `utility.hpp`) applies `extrinsicRot` to raw
accelerometer/gyroscope readings and `extrinsicRPY` to the orientation field, rotating
all three from the IMU's own frame into the lidar's frame before anything else
touches them - **this is the physical lidar-IMU mounting rotation**, not an internal
IMU-chip raw-axis quirk (an easy thing to get backwards, since FAST-LIO's analogous
`extrinsic_R` parameter has different semantics: it only ever rotates raw accel/gyro
*vectors*, never treats an IMU-derived orientation as a Euler-angle pose to be
incrementally fused, so a lesson from that migration didn't transfer directly).

Getting this wrong was the direct cause of two real bugs found and fixed during this
migration:
1. Setting it to identity (misreading `imuConverter()` as chip-quirk correction, not
   mounting correction) broke gravity compensation entirely, producing runaway
   `"Large velocity, reset IMU-preintegration!"` divergence the moment the robot
   moved.
2. Setting it correctly to match the *old* Mid-360 mount's 180°-about-X flip
   (`diag(1,-1,-1)`) was geometrically right for that mount, but seeded
   `mapOptmization`'s first keyframe with `imu_roll_init = -pi` (confirmed directly
   via `ros2 topic echo /lio_sam/deskew/cloud_info --field imu_roll_init`) - a real
   Euler-angle singularity that LIO-SAM's roll/pitch IMU-fusion step
   (`mapOptmization.cpp`, naive angle averaging) is not built to handle, causing
   catastrophic pose divergence (hundreds of meters within 1-2 frames) even at rest.

The actual root fix was in `lidar.xacro` itself, not the yaml: **the lidar mount is
no longer physically flipped 180°**. The old Mid-360 mount used a 180°-about-X flip
to point its asymmetric FOV mostly downward; OS1's FOV is symmetric (±22.5°), so no
such trick is needed at all - a level, upright mount already points half the beams up
and half down. `extrinsicRot`/`extrinsicRPY` are correctly identity now because the
physical mount really is unrotated, not because of a misreading of `imuConverter()`.
**This is foundational knowledge for any future sensor swap in this repo**: if a new
lidar's FOV needs a downward bias, prefer encoding it directly in the `gpu_lidar`
sensor's native vertical `min_angle`/`max_angle` (as done here) over a physical mount
rotation, to avoid this whole class of bug.

### Note on lidar self-occlusion (OS1 mount)

Re-derived from scratch for OS1 - the old Mid-360 self-filter box does not transfer
(different FOV shape, different mount height, no mount rotation). Confirmed via
direct point-cloud inspection: a self-hit cluster at range 0.24-0.50m, z tightly
banded to -0.117 to -0.078m in the lidar's own frame (consistent with grazing the
chassis's flat top surface at roughly constant depth below the sensor across many
azimuths), with a clear gap before legitimate environment returns resume past 0.6m.
Fixed the same way as Mid-360's: an axis-aligned self-filter box in the adapter
(`gz_lidar_to_ouster.cpp`'s `self_filter_*` parameters), bounds set to the observed
cluster extent plus a 3cm margin. Mount height (`laser_joint` z=0.15 in `lidar.xacro`)
is much lower than Mid-360's 0.25 needed - OS1's shallowest downward angle (-22.5°) is
far less steep than Mid-360's (-52° after its old flip), so a lower mount already
clears most of the chassis; the filter box handles what's left.

If you change the lidar mount position/height or the chassis dimensions, recompute
both the mount clearance and the `self_filter_*` parameters
(`lio_sam_bringup`'s `gz_lidar_to_ouster` node) to match.

### Known limitation: `terrain_test.world` and open terrain

LIO-SAM's front end (`featureExtraction`'s LOAM-style discrete edge/surface feature
extraction) needs geometrically distinguishing, non-coplanar structure nearby to
constrain x/y/yaw well - unlike FAST-LIO's direct point-to-plane/edge nearest-neighbor
matching against a continuously-growing ikd-tree, which tolerated the same sparse
`terrain_test.world` spawn area without issue. This is a genuine architectural
difference, not a bug in either implementation.

Symptom, confirmed empirically: on a fresh `terrain_test.world` launch, LIO-SAM's pose
drifted noticeably even with the robot completely stationary, despite normal feature
counts (89 corner / 1352 surface, both well above the configured minimums) - the
problem wasn't too few features, but too many *coplanar* ones (open flat ground)
relative to features that actually constrain horizontal motion. Mitigated by adding
small, non-repetitive 3D geometry near the spawn point (`spawn_wall_left/right`,
`spawn_box_1/2/3`, `spawn_anchor_wall` in `terrain_test.world`) - this reduced
at-rest drift from an unbounded ~30m runaway to a much smaller, bounded amount, but
**did not fully eliminate residual drift during the ramp climb itself** (position
error ~26% over the climb in the last measured run, versus FAST-LIO's 0.85° pitch
error in the same scenario). `house.world` (used for the basic-odometry, wheel-slip,
and wall-collision tests above) has no equivalent issue - it's furniture-dense enough
that this degeneracy never surfaces there.

**This needs more work before the ramp scenario specifically can be called
production-ready** - options for a follow-up, in rough order of effort: add more
substantial permanent geometry along the ramp run-up itself (not just near spawn),
tune `edgeThreshold`/`surfThreshold` to prefer more discriminating features over raw
count, or re-enable and tune loop closure (currently `loopClosureEnableFlag: false` in
`lio_sam_params.yaml` - disabled as a precaution while chasing this bug, never
confirmed to be either the cause or irrelevant, so it's an open question worth
revisiting once the underlying degeneracy is fully resolved). The basic-odometry,
wheel-slip, and wall-collision results above are the ones that matter most for a
typical indoor delivery/navigation use case, and those are solid.
