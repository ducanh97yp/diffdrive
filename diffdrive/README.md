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

To save the built map and relocalize against it later without re-mapping from
scratch, see [step 3](#3-save-the-map-and-localize-on-a-saved-map) below.

## 3. Save the map and localize on a saved map

LIO-SAM itself has no localization-only mode - it only ever runs live SLAM. It does
ship a save-only `lio_sam/save_map` service, but nothing to relocalize against a
saved map, so `lio_sam_bringup` adds a small ICP-based node
(`pcd_map_localizer`, `lio_sam_bringup/src/pcd_map_localizer.cpp`) for that second
half.

### 3a. Save the map

While step 2's `lio_sam_slam_launch.py` is still running:

```bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
  "{resolution: 0.2, destination: '/ws_ros2_test/maps/lio_sam_house'}"
```

**`destination` is appended directly after `$HOME`, not treated as an absolute
path** - this is upstream LIO-SAM behavior
(`mapOptmization.cpp`'s save-map service handler does
`std::getenv("HOME") + req->destination`, no separator inserted). Passing a full
path like `/home/andy/ws_ros2_test/maps/lio_sam_house` here silently saves into
`/home/andy/home/andy/ws_ros2_test/maps/lio_sam_house` instead - always give
`destination` as the part *after* `$HOME` (leading `/`, no `$HOME` prefix), as in
the example above. Confirmed by direct testing.

This writes `GlobalMap.pcd` (what step 3b loads), `CornerMap.pcd`, `SurfMap.pcd`,
`trajectory.pcd`, `transformations.pcd` into that directory.

### 3b. Localize on the saved map

```bash
ros2 launch lio_sam_bringup lio_sam_localization_launch.py \
  map_pcd_path:=/home/andy/ws_ros2_test/maps/lio_sam_house/GlobalMap.pcd \
  initial_x:=0.0 initial_y:=0.0 initial_z:=0.0 initial_yaw:=0.0
```

Identical to `lio_sam_slam_launch.py` (same Gazebo bringup, same 20s settle delay,
same 4 LIO-SAM nodes still running full local SLAM into a fresh `odom` frame) except
the static identity `map -> odom` link is replaced by `pcd_map_localizer`: every
`update_period` seconds (default 1.0s) it ICP-registers the current
`/ouster_points` scan (transformed into this run's `odom` frame) against the loaded
`GlobalMap.pcd`, and broadcasts the resulting `map -> odom` correction continuously
at 50Hz. Rejects and keeps the previous estimate if `icp.getFitnessScore()` exceeds
`fitness_score_threshold` (default 0.5).

`initial_x/y/z/yaw` seed the ICP search - important because a fresh run's `odom`
frame starts wherever the robot spawns *this* time, which is rarely where mapping
originally started. Get this too wrong and ICP will converge to the wrong place in
the map (or not converge at all). Nudge it live instead via RViz's "2D Pose
Estimate" tool (publishes `/initialpose`, the same mechanism AMCL uses) if the
initial guess was off.

RViz shows the loaded prior map on the `/prior_map` topic (latched, so it appears
regardless of whether RViz was already open when the node started) - it's published
at full resolution by default, separate from the coarser copy used internally for
ICP, so the map doesn't look sparse/blurry compared to what was actually saved.

`/prior_map` used to be published from inside the constructor with a stamp of
`get_clock()->now()` - with `use_sim_time`, that read as exactly 0 (the node's
`/clock` subscription hasn't received its first message yet that early), and RViz's
PointCloud2/TF display treats a 0 stamp as a real, very-old time rather than
"latest," which showed up as transform/sync errors in the Displays panel right after
launch. Fixed by polling on a wall timer until `get_clock()->now()` actually reports
a nonzero time before publishing once, instead of stamping too early.

`pcd_map_localizer` also projects the loaded map down to a 2D `nav_msgs/OccupancyGrid`
and publishes it (once, latched) on `/map` - the same topic and QoS pattern
`nav2_map_server` itself uses, so Nav2's `static_layer` (see
[step 4](#4-autonomous-navigation-with-nav2)) and RViz's Map display both pick it up
without any extra bridging node. Projection is a simple pcd2pgm-style height slice
(`map2d_z_min`/`map2d_z_max`, default 0.05-1.5m): any grid cell containing a point in
that band is marked occupied, everything else in the padded XY bounding box
(`map2d_padding`, default 1.0m) is marked free - no ray tracing/occlusion reasoning,
good enough for a static costmap layer but not survey-grade. `map2d_resolution`
(default 0.05m) sets the grid cell size.

Tuning knobs, exposed as both `pcd_map_localizer` ROS params and
`lio_sam_localization_launch.py` launch arguments: `icp_update_period` (default
**0.2s** - deliberately tightened from the node's own 1.0s fallback default),
`icp_max_correspondence_distance` (default 1.0m), `icp_fitness_score_threshold`
(default 0.5), `icp_voxel_leaf_scan` (default 0.2m), `icp_degenerate_eigenvalue_ratio`
(default 0.45), `icp_max_correction_per_update` (default 0.3m) - see
[degeneracy-aware correction](#degeneracy-aware-icp-correction) below for what those
two do - and `tf_tolerance` (default 0.2s, see next paragraph). `voxel_leaf_map` (ICP
target downsample, default 0.3m) and `voxel_leaf_display` (downsample for
`/prior_map` only, default `0.0` = full resolution) are node-only params, not yet
promoted to launch arguments.

**`map -> odom` is stamped `tf_tolerance` seconds into the future**, the same trick
AMCL uses for its own map->odom broadcast - not just for robustness, this was
confirmed necessary: without it, sending a Nav2 goal accepted fine but the robot
never moved. `controller_server` threw `Exception in transformPose: Lookup would
require extrapolation into the future` on `map -> odom` (a lookup at "now", or
slightly after by the time a request round-trips through another node, landing
between two of `pcd_map_localizer`'s periodic broadcasts), aborted `follow_path`,
and `bt_navigator` reported "Goal failed" - `/cmd_vel` kept publishing at its normal
rate throughout, just with an all-zero `Twist`, which is what made this look like a
"the robot ignores the command" problem rather than a TF timing one. Confirmed fixed
end-to-end: same goal, same map, robot actually drove from its start pose to within
the controller's goal tolerance of the requested `(3.0, 0.0)`.

### Degeneracy-aware ICP correction

Plain point-to-point ICP has a real failure mode: its fitness score measures *how
well the final alignment fits*, not *whether each direction was actually
constrained enough to trust*. First discovered on `terrain_test.world`'s spawn
corridor (between `spawn_wall_left`/`spawn_wall_right`, two long parallel walls
added to fix LIO-SAM's own front-end degeneracy there): with a fresh localization
run's initial identity guess, ICP locked onto a wrong alignment with a
*confident-looking* low fitness score (~0.015-0.02) but a spurious ~1.4m offset
along the corridor axis - sliding along it barely changes point-to-point distances
to either wall, so a wrong offset scores nearly as well as the correct one.

Fixed in `pcd_map_localizer.cpp`'s `applyDegeneracyAwareCorrection()`, mirroring
LOAM/LEGO-LOAM's own degeneracy handling: build an approximate translation
information matrix from the surface normals of the map points each scan point
corresponds to after alignment (map normals are precomputed once at load time via
`pcl::NormalEstimation`), eigen-decompose it, and only accept the ICP-computed
translation change along eigenvectors whose eigenvalue isn't small relative to the
largest (`icp_degenerate_eigenvalue_ratio`, default 0.45 - tuned from real
measurements: the corridor's genuinely weak axis measured ratio ~0.30, a normal
axis ~0.67-0.79). The rejected directions keep whatever the previous estimate had
there instead of trusting a direction the scan couldn't actually see. A second,
independent safeguard (`icp_max_correction_per_update`, default 0.3m) hard-caps how
far a single update may move `map -> odom` regardless of the eigenvalue check -
added after measuring that a *weakly* (not fully singularly) constrained axis can
still produce an occasional large single-frame jump (~1.25m in one 0.2s update, in
a manual `/initialpose`-reseed stress test) that a ratio threshold alone can miss.

**Tested manually end-to-end** (build map from a short drive, save, relaunch fresh
into localization mode with the same spawn pose):

- `house.world` (furniture-dense, well-conditioned geometry): no change in behavior
  from before the degeneracy fix - ICP converged immediately (fitness ~0.01-0.02 at
  rest), `map -> odom` stayed stable (~1-2cm offset) while driving, and the
  degeneracy check never flagged any axis (as expected - all three stay
  well-constrained indoors). No regression.
- `terrain_test.world` spawn corridor: **the originally-reported spurious lock is
  fixed** - confirmed over multiple fresh launches, `map -> odom` now converges to
  the expected small (~1-8cm) offset instead of the ~1.4m wrong lock, with the log
  correctly showing `1/3 translation axis(es) degenerate` throughout (the corridor
  axis is being identified and excluded, exactly as intended). A deliberately
  adversarial stress test (forcing a wrong ~1.4m seed via `/initialpose` inside the
  corridor) showed the fix does **not** guarantee recovery from an already-wrong
  seed in genuinely ambiguous geometry - that is a harder problem (global
  relocalization / multi-hypothesis tracking) than what a local, single-hypothesis
  ICP correction can solve, and is out of scope here. It does not affect normal
  startup, which always begins from `initial_x/y/z/yaw` (default identity), not an
  adversarial seed.
- `terrain_test.world` full ramp+platform drive: ground-truth-vs-estimate error was
  ~4cm right after spawn, growing to over 1m by the top of the ramp/platform. This
  is **not a bug in the correction logic** - the log confirms the degeneracy check
  correctly flags **2 of 3** translation axes as degenerate for most of the ramp
  and the platform, because both are close to featureless flat ground: a flat
  surface constrains exactly one direction (its normal) and leaves both in-plane
  directions unconstrained, whether tilted (ramp) or level (platform). LIO-SAM's
  own front-end drift (the same root cause documented in the
  [known limitation](#known-limitation-terrain_testworld-and-open-terrain) below)
  therefore goes uncorrected through this stretch - correctly so: there is not
  enough 3D structure there for *any* point-cloud method to safely correct position,
  and reporting an uncorrected-but-honest estimate is preferable to a
  confidently-wrong one. Closing this residual gap needs more permanent 3D
  structure along the ramp/platform (same fix direction the known-limitation
  section already recommends for LIO-SAM's own front end) - it is not something
  more ICP-side cleverness alone can solve.

Not yet tested: spawning from a genuinely different pose than the original mapping
run (exercises the `initial_x/y/yaw`/`/initialpose` seeding path under normal,
non-adversarial conditions), or a large multi-room map like the "very good" one
referenced in this repo's own testing notes.

## 4. Autonomous navigation with Nav2

Nav2 (`ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`) is installed as part of
[Prerequisites](#prerequisites) above. Run this alongside step 2 (mapping) or step 3
(localizing on a saved map) - **which config file to pass differs between the two**:

```bash
# Alongside step 2 (lio_sam_slam_launch.py) - no prior map, live-built rolling costmap:
ros2 launch lio_sam_bringup nav2_bringup_launch.py

# Alongside step 3 (lio_sam_localization_launch.py) - static map + live obstacles:
ros2 launch lio_sam_bringup nav2_bringup_launch.py \
  params_file:=/home/andy/ws_ros2_test/install/lio_sam_bringup/share/lio_sam_bringup/config/nav2_params_localization.yaml
```

LIO-SAM (not AMCL) provides localization directly. The TF chain is `map -> odom`
(static identity from step 2's launch file, or the dynamic ICP correction from step
3's `pcd_map_localizer` if localizing on a saved map) `-> base_link` (dynamic, broadcast by
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
a `PointCloud2` source. Local costmap's voxel layer covers up to z=2.0m (`z_voxels: 16`,
`z_resolution: 0.125`) - raised from an initial 0.8m after an "out of map bounds"
warning appeared with the robot elevated on `terrain_test.world`'s ramp platform;
`origin_z` is a fixed world-z, not robot-relative, so this ceiling is still finite and
won't follow the robot onto arbitrarily tall structures. `z_voxels` is capped at 16,
not raised further - `nav2_costmap_2d::VoxelLayer` packs the whole Z column into a
single 16-bit mask, so anything above 16 isn't slow, it's an unsupported
configuration that silently prevented `controller_server` from ever activating
(confirmed directly: `nav2_bringup_launch.py` never came up with `z_voxels: 40`, in
*either* mode - a real, pre-existing bug found and fixed alongside the `/map` work
below, not something introduced by it).

`global_costmap` behavior now depends on which params file you launched with:
- **`nav2_params.yaml`** (step 2 / mapping): `rolling_window: true`, no `static_layer`
  - there's no prior map to load, so the costmap is built entirely live from
  `/ouster_points` via `obstacle_layer`, centered on and following the robot (a
  non-rolling costmap without a static layer instead defaults to a small fixed box
  anchored at world origin - confirmed via Nav2's own "out of map bounds" warning
  once the robot drove outside it).
- **`nav2_params_localization.yaml`** (step 3 / localization): non-rolling, with a
  `static_layer` reading `/map` (a 2D occupancy grid `pcd_map_localizer` projects
  from the loaded `GlobalMap.pcd` - see [step 3b](#3b-localize-on-the-saved-map))
  underneath the same live `obstacle_layer`. The costmap auto-sizes itself to match
  the received map's own extent (confirmed: a 763x1002-cell map produced a
  763x1002-cell costmap with the same origin) - the standard Nav2 pattern for a
  known-map robot. **This is what actually answers "why doesn't Nav2 show a 2D map"**
  when localizing on a saved map: without this file, `nav2_params.yaml`'s
  `global_costmap` has no `static_layer` at all (by design - a pure-SLAM run has no
  map to load), and nothing else ever published `/map`, so RViz's Map display
  (subscribed to `/map`, `Durability Policy: Transient Local`) stayed empty
  regardless of localization actually working underneath. Confirmed working
  end-to-end: `/map` publishes with the map's real wall geometry, and the resulting
  costmap shows real occupied/inflated cells matching it (spot-checked via `ros2
  topic echo`, not just "no errors").

Send a goal either through RViz's "2D Nav Goal" tool (Fixed Frame `map`), or directly:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
```

Config lives in `lio_sam_bringup/config/nav2_params.yaml` (and
`nav2_params_localization.yaml`, its counterpart for step 3 - kept as a separate file
rather than one shared file, since a `static_layer` present but never fed `/map`
during a pure-SLAM run would leave that layer, and therefore the whole costmap,
stuck "not current," blocking planning). The robot footprint is
approximated as a 0.2m radius circle - fine for open-space navigation, but tune this
(or switch to an explicit `footprint` polygon) if it needs to fit through tight gaps.

## Package layout

- **`diff_description`** - robot URDF/xacro (Ouster OS1 lidar + industrial IMU),
  Gazebo worlds, ros2_control config, and the base Gazebo bringup launch file
  (`rviz_joint_control.launch.py`) everything else includes.
- **`lio_sam_bringup`** - the Gazebo/Nav2 integration built around LIO-SAM: the
  `gz_lidar_to_ouster` point cloud adapter node, the `pcd_map_localizer` ICP
  relocalization node (see [step 3](#3-save-the-map-and-localize-on-a-saved-map)),
  and SLAM/localization/Nav2 launch files + configs.
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

The same parallel-wall geometry near spawn (added here to help LIO-SAM's front end)
turned out to cause an analogous degeneracy for
[`pcd_map_localizer`'s ICP correction](#degeneracy-aware-icp-correction) - point-to-point
ICP fitness score alone can't distinguish "well localized" from "confidently wrong
along an unconstrained axis." That specific spurious lock is now fixed there (see
that section's test notes), but the fix is honest rather than magic: it makes
`pcd_map_localizer` correctly *decline* to correct position on this world's mostly-flat
ramp and platform (both are close to featureless, so 2 of 3 translation axes are
genuinely unconstrained there) rather than silently accept LIO-SAM's own drift as
correct - so this world still isn't a case where the *combined* system holds accurate
position through the ramp climb, independent of the front-end issue documented above.
Both issues share the same underlying cause and the same fix direction: more permanent
3D structure along the ramp/platform, not along the spawn corridor alone.

### `house.world`'s small ramp + ridges

`house.world` (`diff_description/worlds/house.world`) now also has a small
up/flat/down ramp course with two low ridges ("gờ"), for stability testing that
doesn't require switching to `terrain_test.world` - unlike that world's 8°/3m ramp
(added specifically to stress LIO-SAM's front end, see the known limitation above),
this one is deliberately gentle: 6°, 1m run per side, ~10.5cm peak height, two 2cm
ridges on the flat top. It's placed at `x=5.3` to `x~7.989`, `y` within `±1.25`,
confirmed clear of both `house.world`'s own furniture and the existing
`slippery_patch` fixture (which ends at `x=5`) by direct drive-through testing before
adding anything.

Unlike `terrain_test.world`'s ramp (one-way, ends in an open platform used for other
tests), this one goes **up, across both ridges, and back down** to ground level - a
repeatable climb/cross/descend cycle rather than a one-off event, better suited to
the kind of repeated stability evaluation this was added for.

**Tested directly** (teleporting the robot to just past the pre-existing
`slippery_patch` via `gz service -s /world/default/set_pose`, to isolate this new
geometry from that separate, already-flaky fixture - see caveat below): climbed
cleanly, both ridges produced the expected momentary nose-up pitch as the front
wheels crossed each one and nothing else (no roll, no lateral drift), descended
cleanly, ended flat and level. Confirms the ramp math itself (reused directly from
`terrain_test.world`'s already-proven formula, just recomputed for a shorter run and
shallower angle) is sound.

**Testing caveat, in the interest of not overclaiming**: later re-tests in this same
long test session became unreliable - not tied to any specific change, but general
degradation from ~25+ Gazebo launch/kill cycles in one sitting (elevated leftover
process count, accumulated `/dev/shm` segments from repeated DDS shared-memory
transport setup/teardown, and `gzserver` itself segfaulting on shutdown twice,
non-deterministically - it shut down cleanly just as often). The pre-existing
`slippery_patch` also became intermittently unable to let the robot coast through it
during these later attempts, despite working fine both earlier in this same session
and in this specific fix's own clean verification run - i.e. this instability
predates and is independent of the ramp addition, not evidence against it. If you hit
similar flakiness, a fresh terminal/session (or a machine restart) is the fix, not a
change to the world file.
