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
#include <pcl_conversions/pcl_conversions.h>

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

    voxelDownsample(raw_map, voxel_leaf_display_);

    // Nothing else in localization mode ever publishes the prior map - mapOptimization
    // in this run only ever builds a fresh, near-empty local map, so without this RViz
    // has nothing to show for "the map" until the robot has re-driven the whole area.
    // Transient-local so RViz gets it even if it subscribes after this one-shot publish.
    prior_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "prior_map", rclcpp::QoS(1).transient_local().reliable());
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*raw_map, map_msg);
    map_msg.header.frame_id = map_frame_;
    map_msg.header.stamp = get_clock()->now();
    prior_map_pub_->publish(map_msg);

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

    std::lock_guard<std::mutex> lock(estimate_mutex_);
    map_T_odom_ = icp.getFinalTransformation();
    RCLCPP_INFO(get_logger(), "pcd_map_localizer: map->odom updated, fitness=%.3f", fitness);
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
    tf_msg.header.stamp = get_clock()->now();
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = odom_frame_;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::string map_pcd_path_, map_frame_, odom_frame_, scan_topic_;
  double update_period_, voxel_leaf_map_, voxel_leaf_scan_, voxel_leaf_display_;
  double max_correspondence_distance_, fitness_score_threshold_;
  int max_iterations_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  Eigen::Matrix4f map_T_odom_ = Eigen::Matrix4f::Identity();
  std::mutex estimate_mutex_;
  rclcpp::Time last_update_{0, 0, RCL_ROS_TIME};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr prior_map_pub_;
  rclcpp::TimerBase::SharedPtr broadcast_timer_;
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
