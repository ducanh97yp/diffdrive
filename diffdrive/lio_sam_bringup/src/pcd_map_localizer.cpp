// Global localization against a LIO-SAM map saved via `lio_sam/save_map`
// (typically GlobalMap.pcd). LIO-SAM itself has no localization-only mode - it
// only ever runs live SLAM - so this node lets a fresh LIO-SAM run (still doing
// its own local odometry/mapping, unchanged) be pinned onto a previously built
// map: it periodically ICP-registers the current scan (transformed into this
// run's "odom" frame) against the saved map cloud, and broadcasts the resulting
// map->odom correction. This replaces lio_sam_slam_launch.py's static identity
// map->odom link, the same way AMCL replaces a static map->odom for 2D nav.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigenvalues>
#include <cmath>
#include <limits>
#include <mutex>

class PcdMapLocalizer : public rclcpp::Node
{
public:
  PcdMapLocalizer() : Node("pcd_map_localizer"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    map_pcd_path_ = declare_parameter<std::string>("map_pcd_path", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "ouster_points");
    update_period_ = declare_parameter<double>("update_period", 1.0);
    voxel_leaf_map_ = declare_parameter<double>("voxel_leaf_map", 0.3);
    voxel_leaf_scan_ = declare_parameter<double>("voxel_leaf_scan", 0.2);
    voxel_leaf_display_ = declare_parameter<double>("voxel_leaf_display", 0.0);
    max_correspondence_distance_ = declare_parameter<double>("max_correspondence_distance", 1.0);
    fitness_score_threshold_ = declare_parameter<double>("fitness_score_threshold", 0.5);
    max_iterations_ = declare_parameter<int>("max_iterations", 50);
    // 0.45 is tuned from real data (see pcd_map_localizer.cpp git history/README): a
    // genuinely weak axis (parallel-wall corridor) measured ~0.30, a normally-constrained
    // one ~0.67-0.79 - comfortable margin on both sides.
    degenerate_eigenvalue_ratio_ = declare_parameter<double>("degenerate_eigenvalue_ratio", 0.45);
    min_degeneracy_correspondences_ = declare_parameter<int>("min_degeneracy_correspondences", 20);
    max_correction_per_update_ = declare_parameter<double>("max_correction_per_update", 0.3);
    map2d_resolution_ = declare_parameter<double>("map2d_resolution", 0.05);
    map2d_z_min_ = declare_parameter<double>("map2d_z_min", 0.05);
    map2d_z_max_ = declare_parameter<double>("map2d_z_max", 1.5);
    map2d_padding_ = declare_parameter<double>("map2d_padding", 1.0);
    tf_tolerance_ = declare_parameter<double>("tf_tolerance", 0.2);
    double init_x = declare_parameter<double>("initial_x", 0.0);
    double init_y = declare_parameter<double>("initial_y", 0.0);
    double init_z = declare_parameter<double>("initial_z", 0.0);
    double init_yaw = declare_parameter<double>("initial_yaw", 0.0);

    if (map_pcd_path_.empty())
    {
      RCLCPP_FATAL(get_logger(), "map_pcd_path parameter is required (path to a saved LIO-SAM GlobalMap.pcd)");
      throw std::runtime_error("map_pcd_path not set");
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_pcd_path_, *raw_map) < 0)
    {
      RCLCPP_FATAL(get_logger(), "Failed to load map cloud from %s", map_pcd_path_.c_str());
      throw std::runtime_error("failed to load map_pcd_path");
    }

    // ICP only needs a coarse voxel grid for fast correspondence search (voxel_leaf_map,
    // default 0.3m) - publishing that same coarse cloud to RViz made the displayed map
    // look sparse/blurry compared to what was actually saved. Keep the two separate:
    // map_cloud_ (coarse, ICP target) vs. what goes out on /prior_map (full resolution
    // by default, only downsampled if voxel_leaf_display is set - useful if the raw PCD
    // is too large for RViz to render smoothly).
    map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>(*raw_map));
    voxelDownsample(map_cloud_, voxel_leaf_map_);
    RCLCPP_INFO(get_logger(), "Loaded map cloud %s (%zu points, %zu after ICP downsample)", map_pcd_path_.c_str(),
                raw_map->size(), map_cloud_->size());

    // Surface normals + a KD-tree over map_cloud_, used after every ICP alignment to
    // check *which directions* the correction was actually constrained along (see
    // checkDegeneracy() below) - a corridor of parallel walls can give ICP a
    // confident-looking low fitness score for a translation that's completely
    // unconstrained along the corridor axis, since sliding along it barely changes
    // point-to-point distances to either wall. Point-to-point ICP alone can't tell
    // the difference; this can.
    map_kdtree_.setInputCloud(map_cloud_);
    map_normals_.reset(new pcl::PointCloud<pcl::Normal>());
    {
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud(map_cloud_);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr normal_tree(new pcl::search::KdTree<pcl::PointXYZ>());
      ne.setSearchMethod(normal_tree);
      ne.setKSearch(15);
      ne.compute(*map_normals_);
    }

    // Nav2's global costmap in this repo has no static_layer (nav2_params.yaml is shared
    // with pure-SLAM runs, which have no prior map to load), and nothing anywhere
    // publishes /map - so Nav2's own RViz view (a separate rviz2 instance/config from
    // LIO-SAM's, launched by nav2_bringup_launch.py) always showed an empty Map display,
    // even while localizing on a saved map. Build a 2D occupancy grid from the same
    // raw_map (full resolution, before display downsampling below) by slicing a height
    // band and marking any cell containing a point in that band as occupied - the same
    // approach common pcd2pgm-style converters use. Pair with
    // nav2_params_localization.yaml's static_layer to actually feed Nav2's costmap.
    nav_msgs::msg::OccupancyGrid occupancy_grid = buildOccupancyGrid(*raw_map);
    RCLCPP_INFO(get_logger(), "Built 2D occupancy grid %ux%u @ %.2fm/cell from z in [%.2f, %.2f]",
                occupancy_grid.info.width, occupancy_grid.info.height, map2d_resolution_, map2d_z_min_, map2d_z_max_);

    voxelDownsample(raw_map, voxel_leaf_display_);

    // Nothing else in localization mode ever publishes the prior map - mapOptimization
    // in this run only ever builds a fresh, near-empty local map, so without this RViz
    // has nothing to show for "the map" until the robot has re-driven the whole area.
    // Transient-local so RViz gets it even if it subscribes after this one-shot publish.
    prior_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "prior_map", rclcpp::QoS(1).transient_local().reliable());
    // map_subscribe_transient_local in nav2_params_localization.yaml's static_layer expects
    // exactly this QoS pattern - it's how nav2_map_server itself publishes /map.
    map2d_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local().reliable());
    // Deferred instead of publishing right here: with use_sim_time, get_clock()->now()
    // reads 0 until the node's /clock subscription has actually received and processed a
    // message, which can't have happened yet this early in the constructor (confirmed
    // empirically - stamped sec=0 when published directly here) and, empirically, is not
    // reliably done within a fixed short delay either (DDS discovery for that subscription
    // isn't instant). RViz's PointCloud2/Map displays treat a stamp of 0 as a real,
    // very-old time rather than "latest", which can show as a transform/sync error in the
    // Displays panel. Poll on a wall timer (unaffected by sim time itself) until the clock
    // actually reports a nonzero time, then publish once and stop - correct regardless of
    // how long that takes, instead of gambling on a fixed delay.
    auto raw_map_for_publish = raw_map;
    publish_prior_map_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this, raw_map_for_publish, occupancy_grid]() {
      if (get_clock()->now().nanoseconds() == 0) return;
      sensor_msgs::msg::PointCloud2 map_msg;
      pcl::toROSMsg(*raw_map_for_publish, map_msg);
      map_msg.header.frame_id = map_frame_;
      map_msg.header.stamp = get_clock()->now();
      prior_map_pub_->publish(map_msg);

      nav_msgs::msg::OccupancyGrid grid_msg = occupancy_grid;
      grid_msg.header.stamp = get_clock()->now();
      grid_msg.info.map_load_time = get_clock()->now();
      map2d_pub_->publish(grid_msg);

      publish_prior_map_timer_->cancel();
    });

    {
      Eigen::Affine3d init = Eigen::Translation3d(init_x, init_y, init_z) *
                             Eigen::AngleAxisd(init_yaw, Eigen::Vector3d::UnitZ());
      std::lock_guard<std::mutex> lock(estimate_mutex_);
      map_T_odom_ = init.matrix().cast<float>();
    }

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PcdMapLocalizer::scanCallback, this, std::placeholders::_1));

    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", rclcpp::SystemDefaultsQoS(),
        std::bind(&PcdMapLocalizer::initialPoseCallback, this, std::placeholders::_1));

    broadcast_timer_ = create_wall_timer(std::chrono::milliseconds(20),
                                          std::bind(&PcdMapLocalizer::broadcastTransform, this));

    RCLCPP_INFO(get_logger(), "pcd_map_localizer ready, aligning '%s' into map '%s' via ICP every %.2fs",
                scan_topic_.c_str(), map_frame_.c_str(), update_period_);
  }

private:
  static void voxelDownsample(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double leaf)
  {
    if (leaf <= 0.0) return;
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());
    vg.filter(*out);
    cloud = out;
  }

  // Simple pcd2pgm-style projection: mark any cell containing a point within
  // [map2d_z_min_, map2d_z_max_] as occupied, everything else in the padded XY bounding
  // box as free. No ray tracing/occlusion reasoning - matches what similar community
  // conversion tools do, good enough for a static costmap layer, not survey-grade.
  nav_msgs::msg::OccupancyGrid buildOccupancyGrid(const pcl::PointCloud<pcl::PointXYZ>& cloud) const
  {
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    for (const auto& p : cloud.points)
    {
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
      min_x = std::min(min_x, p.x);
      max_x = std::max(max_x, p.x);
      min_y = std::min(min_y, p.y);
      max_y = std::max(max_y, p.y);
    }
    min_x -= static_cast<float>(map2d_padding_);
    min_y -= static_cast<float>(map2d_padding_);
    max_x += static_cast<float>(map2d_padding_);
    max_y += static_cast<float>(map2d_padding_);

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = map_frame_;
    grid.info.resolution = static_cast<float>(map2d_resolution_);
    grid.info.width = static_cast<uint32_t>(std::ceil((max_x - min_x) / map2d_resolution_));
    grid.info.height = static_cast<uint32_t>(std::ceil((max_y - min_y) / map2d_resolution_));
    grid.info.origin.position.x = min_x;
    grid.info.origin.position.y = min_y;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(static_cast<size_t>(grid.info.width) * grid.info.height, 0);

    for (const auto& p : cloud.points)
    {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      if (p.z < map2d_z_min_ || p.z > map2d_z_max_) continue;
      int cx = static_cast<int>((p.x - min_x) / map2d_resolution_);
      int cy = static_cast<int>((p.y - min_y) / map2d_resolution_);
      if (cx < 0 || cy < 0 || cx >= static_cast<int>(grid.info.width) || cy >= static_cast<int>(grid.info.height))
      {
        continue;
      }
      grid.data[static_cast<size_t>(cy) * grid.info.width + static_cast<size_t>(cx)] = 100;
    }
    return grid;
  }

  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    // RViz's "2D Pose Estimate" publishes map_T_base; combine with the current
    // odom_T_base lookup to reseed map_T_odom_, exactly like AMCL's initialpose use.
    geometry_msgs::msg::TransformStamped odom_T_base;
    try
    {
      odom_T_base = tf_buffer_.lookupTransform(odom_frame_, "base_link", tf2::TimePointZero);
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN(get_logger(), "initialpose: could not look up %s->base_link (%s)", odom_frame_.c_str(), ex.what());
      return;
    }

    Eigen::Affine3d map_T_base;
    tf2::fromMsg(msg->pose.pose, map_T_base);
    Eigen::Isometry3d odom_T_base_iso = tf2::transformToEigen(odom_T_base);

    std::lock_guard<std::mutex> lock(estimate_mutex_);
    map_T_odom_ = (map_T_base * odom_T_base_iso.inverse()).matrix().cast<float>();
    RCLCPP_INFO(get_logger(), "pcd_map_localizer: reseeded map->odom from /initialpose");
  }

  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    rclcpp::Time now = msg->header.stamp;
    if (last_update_.nanoseconds() != 0 && (now - last_update_).seconds() < update_period_)
    {
      return;
    }

    geometry_msgs::msg::TransformStamped odom_T_lidar;
    try
    {
      odom_T_lidar = tf_buffer_.lookupTransform(odom_frame_, msg->header.frame_id, msg->header.stamp,
                                                 rclcpp::Duration::from_seconds(0.2));
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "pcd_map_localizer: TF %s->%s unavailable (%s)",
                            odom_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *scan);
    if (scan->empty()) return;

    Eigen::Isometry3d odom_T_lidar_iso = tf2::transformToEigen(odom_T_lidar);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_in_odom(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*scan, *scan_in_odom, odom_T_lidar_iso.matrix().cast<float>());
    voxelDownsample(scan_in_odom, voxel_leaf_scan_);
    if (scan_in_odom->size() < 30) return;

    Eigen::Matrix4f guess;
    {
      std::lock_guard<std::mutex> lock(estimate_mutex_);
      guess = map_T_odom_;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(scan_in_odom);
    icp.setInputTarget(map_cloud_);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
    icp.setMaximumIterations(max_iterations_);
    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned, guess);

    last_update_ = now;
    if (!icp.hasConverged())
    {
      RCLCPP_WARN(get_logger(), "pcd_map_localizer: ICP did not converge, keeping previous map->odom estimate");
      return;
    }
    double fitness = icp.getFitnessScore();
    if (fitness > fitness_score_threshold_)
    {
      RCLCPP_WARN(get_logger(), "pcd_map_localizer: ICP fitness %.3f above threshold %.3f, rejecting update", fitness,
                  fitness_score_threshold_);
      return;
    }

    Eigen::Matrix4f icp_result = icp.getFinalTransformation();
    Eigen::Matrix4f accepted;
    if (!applyDegeneracyAwareCorrection(aligned, guess, icp_result, accepted))
    {
      RCLCPP_WARN(get_logger(),
                  "pcd_map_localizer: too few normal-bearing correspondences (<%d) to check degeneracy, "
                  "rejecting update",
                  min_degeneracy_correspondences_);
      return;
    }

    std::lock_guard<std::mutex> lock(estimate_mutex_);
    map_T_odom_ = accepted;
    RCLCPP_INFO(get_logger(), "pcd_map_localizer: map->odom updated, fitness=%.3f", fitness);
  }

  // Point-to-point ICP's fitness score measures how well the final alignment fits -
  // not whether each direction was actually constrained enough to trust. A corridor
  // of parallel walls is the textbook failure case: sliding along the corridor axis
  // barely changes point-to-point distances, so ICP can converge "confidently" to a
  // translation that's wrong along that one axis. This mirrors LOAM/LEGO-LOAM's own
  // degeneracy handling: build an approximate translation information matrix from
  // the surface normals at each correspondence (a normal with a strong X component
  // means that correspondence constrains X well; a wall's-worth of Y/Z-only normals
  // contributes nothing to constraining X), eigen-decompose it, and only accept the
  // ICP correction along eigenvectors whose eigenvalue isn't tiny relative to the
  // largest - the rest keeps the previous (pre-ICP) estimate's component instead of
  // trusting a direction the scan couldn't actually see.
  bool applyDegeneracyAwareCorrection(const pcl::PointCloud<pcl::PointXYZ>& aligned, const Eigen::Matrix4f& guess,
                                       const Eigen::Matrix4f& icp_result, Eigen::Matrix4f& accepted)
  {
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    int count = 0;
    std::vector<int> nn_idx(1);
    std::vector<float> nn_sq_dist(1);
    const float max_dist_sq = static_cast<float>(max_correspondence_distance_ * max_correspondence_distance_);
    for (const auto& p : aligned.points)
    {
      if (map_kdtree_.nearestKSearch(p, 1, nn_idx, nn_sq_dist) == 0) continue;
      if (nn_sq_dist[0] > max_dist_sq) continue;
      const pcl::Normal& normal = map_normals_->points[nn_idx[0]];
      if (!std::isfinite(normal.normal_x) || !std::isfinite(normal.normal_y) || !std::isfinite(normal.normal_z))
      {
        continue;
      }
      Eigen::Vector3f n(normal.normal_x, normal.normal_y, normal.normal_z);
      H += n * n.transpose();
      ++count;
    }

    if (count < min_degeneracy_correspondences_) return false;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(H);
    Eigen::Vector3f eigenvalues = solver.eigenvalues();     // ascending
    Eigen::Matrix3f eigenvectors = solver.eigenvectors();
    float max_eig = eigenvalues(2);

    Eigen::Vector3f t0 = guess.block<3, 1>(0, 3);
    Eigen::Vector3f t1 = icp_result.block<3, 1>(0, 3);
    Eigen::Vector3f delta = t1 - t0;

    Eigen::Vector3f accepted_delta = Eigen::Vector3f::Zero();
    int degenerate_axes = 0;
    for (int i = 0; i < 3; ++i)
    {
      bool degenerate = max_eig < 1e-6f || eigenvalues(i) < static_cast<float>(degenerate_eigenvalue_ratio_) * max_eig;
      if (degenerate)
      {
        ++degenerate_axes;
        continue;
      }
      Eigen::Vector3f v = eigenvectors.col(i);
      accepted_delta += v * v.dot(delta);
    }

    if (degenerate_axes > 0)
    {
      RCLCPP_WARN(get_logger(),
                  "pcd_map_localizer: %d/3 translation axis(es) degenerate (eigenvalue ratio < %.2f) - "
                  "rejecting ICP correction along them, keeping previous estimate there",
                  degenerate_axes, degenerate_eigenvalue_ratio_);
    }

    // Belt-and-suspenders on top of the eigenvalue check: measured on a real parallel-wall
    // corridor (terrain_test.world spawn area), the weak axis wasn't a clean numerical
    // singularity (eigenvalue ratio ~0.3, not ~0) - just weakly constrained enough that a
    // single noisy ICP solve still occasionally jumped >1m along it in one 0.2s update, which
    // a purely ratio-based threshold can't reliably distinguish from a large jump that's
    // actually deserved (e.g. right after startup or an /initialpose reseed, both of which
    // *should* be allowed to move a lot). Clamp the magnitude of what a single ICP update may
    // change map->odom by; anything the true state needs beyond that just takes a few more
    // 0.2s cycles instead of one leap of faith.
    float delta_norm = accepted_delta.norm();
    if (delta_norm > static_cast<float>(max_correction_per_update_))
    {
      RCLCPP_WARN(get_logger(), "pcd_map_localizer: capping ICP correction %.3fm -> %.3fm (max_correction_per_update)",
                  delta_norm, max_correction_per_update_);
      accepted_delta *= static_cast<float>(max_correction_per_update_) / delta_norm;
    }

    accepted = Eigen::Matrix4f::Identity();
    accepted.block<3, 3>(0, 0) = icp_result.block<3, 3>(0, 0);
    accepted.block<3, 1>(0, 3) = t0 + accepted_delta;
    return true;
  }

  void broadcastTransform()
  {
    Eigen::Matrix4f m;
    {
      std::lock_guard<std::mutex> lock(estimate_mutex_);
      m = map_T_odom_;
    }
    Eigen::Isometry3d iso(m.cast<double>());

    geometry_msgs::msg::TransformStamped tf_msg = tf2::eigenToTransform(iso);
    // Stamped tf_tolerance_ into the future, same trick AMCL uses for its own
    // map->odom broadcast: a lookup at "now" (or slightly after, by the time a
    // request round-trips through another node) landing between two of our 20ms
    // broadcasts would otherwise throw "Lookup would require extrapolation into the
    // future" - confirmed to actually break Nav2 with this, not just a theoretical
    // risk: controller_server hit exactly this exception on map->odom, aborted
    // follow_path, and the robot never moved despite /cmd_vel still publishing
    // (all zeros) and the goal being accepted.
    tf_msg.header.stamp = get_clock()->now() + rclcpp::Duration::from_seconds(tf_tolerance_);
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = odom_frame_;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::string map_pcd_path_, map_frame_, odom_frame_, scan_topic_;
  double update_period_, voxel_leaf_map_, voxel_leaf_scan_, voxel_leaf_display_;
  double max_correspondence_distance_, fitness_score_threshold_;
  int max_iterations_;
  double degenerate_eigenvalue_ratio_;
  int min_degeneracy_correspondences_;
  double max_correction_per_update_;
  double map2d_resolution_, map2d_z_min_, map2d_z_max_, map2d_padding_;
  double tf_tolerance_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  pcl::PointCloud<pcl::Normal>::Ptr map_normals_;
  pcl::KdTreeFLANN<pcl::PointXYZ> map_kdtree_;
  Eigen::Matrix4f map_T_odom_ = Eigen::Matrix4f::Identity();
  std::mutex estimate_mutex_;
  rclcpp::Time last_update_{0, 0, RCL_ROS_TIME};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr prior_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map2d_pub_;
  rclcpp::TimerBase::SharedPtr broadcast_timer_;
  rclcpp::TimerBase::SharedPtr publish_prior_map_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<PcdMapLocalizer>());
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("pcd_map_localizer"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
