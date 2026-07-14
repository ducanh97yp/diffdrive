#include "lio_sam_bringup/elevation_traversability_layer.hpp"

#include <nav2_costmap_2d/cost_values.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace lio_sam_bringup
{

void ElevationTraversabilityLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node)
  {
    throw std::runtime_error("ElevationTraversabilityLayer: failed to lock node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("point_cloud_topic", rclcpp::ParameterValue(std::string("ouster_points")));
  // 0.6, not 0.0: this layer has no raytraced clearing (see the class-level comment
  // in the header for that tradeoff), so unlike the old ObstacleLayer/VoxelLayer a
  // single self-hit point that leaks past gz_lidar_to_ouster's own self-filter box
  // (x in [-0.24,0.51], y in [-0.31,0.31], axis-aligned so its farthest corner is
  // ~0.6m away) doesn't get corrected next cycle - it gets "fast-fall" instant-
  // adopted and then sits there for a stationary robot until the rolling window
  // happens to shift. Confirmed empirically: with min_range 0.0, the robot saw a
  // lethal blob centered on its own position and could never find a non-colliding
  // trajectory anywhere. 0.6m clears the self-filter box's own bounds with margin.
  declareParameter("obstacle_min_range", rclcpp::ParameterValue(0.6));
  declareParameter("obstacle_max_range", rclcpp::ParameterValue(2.5));
  declareParameter("slope_check_radius_cells", rclcpp::ParameterValue(4));
  declareParameter("caution_slope_deg", rclcpp::ParameterValue(15.0));
  declareParameter("max_slope_deg", rclcpp::ParameterValue(35.0));
  declareParameter("height_alpha_down", rclcpp::ParameterValue(1.0));
  declareParameter("height_alpha_up", rclcpp::ParameterValue(0.02));
  declareParameter("publish_debug_grid", rclcpp::ParameterValue(true));

  node->get_parameter(getFullName("enabled"), enabled_);
  node->get_parameter(getFullName("point_cloud_topic"), point_cloud_topic_);
  node->get_parameter(getFullName("obstacle_min_range"), obstacle_min_range_);
  node->get_parameter(getFullName("obstacle_max_range"), obstacle_max_range_);
  node->get_parameter(getFullName("slope_check_radius_cells"), slope_check_radius_cells_);
  double caution_deg = 15.0, max_deg = 35.0;
  node->get_parameter(getFullName("caution_slope_deg"), caution_deg);
  node->get_parameter(getFullName("max_slope_deg"), max_deg);
  caution_slope_rad_ = caution_deg * M_PI / 180.0;
  max_slope_rad_ = max_deg * M_PI / 180.0;
  node->get_parameter(getFullName("height_alpha_down"), height_alpha_down_);
  node->get_parameter(getFullName("height_alpha_up"), height_alpha_up_);
  node->get_parameter(getFullName("publish_debug_grid"), publish_debug_grid_);

  matchSize();

  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ElevationTraversabilityLayer::cloudCallback, this, std::placeholders::_1));

  if (publish_debug_grid_)
  {
    debug_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        name_ + "/debug_slope_grid", rclcpp::QoS(1));
  }

  // Cells default to NO_INFORMATION until real data arrives (see updateCosts) - no
  // reason to block the whole costmap on this layer specifically being "current".
  current_ = true;

  RCLCPP_INFO(
      node->get_logger(),
      "%s: slope-aware traversability from '%s' (caution %.1f deg, lethal %.1f deg, "
      "radius %d cells)",
      name_.c_str(), point_cloud_topic_.c_str(), caution_deg, max_deg, slope_check_radius_cells_);
}

void ElevationTraversabilityLayer::matchSize()
{
  CostmapLayer::matchSize();
  std::lock_guard<std::mutex> lock(height_mutex_);
  height_.assign(
      static_cast<size_t>(getSizeInCellsX()) * getSizeInCellsY(),
      std::numeric_limits<float>::quiet_NaN());
}

void ElevationTraversabilityLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
  Costmap2D::updateOrigin(new_origin_x, new_origin_y);
  // Rolling costmaps (local_costmap, and global_costmap in pure-SLAM mode) recenter
  // every cycle via this hook, not matchSize() (which only fires once at startup) -
  // reset instead of trying to shift/remap the height grid: cheap at this grid size
  // (<=3600 cells) and any gap is repopulated within ~0.2-0.3s at the point cloud's
  // ~10Hz rate. A non-rolling costmap (global_costmap in localization mode, sized to
  // a loaded static map) never calls this, so its height grid accumulates for the
  // whole run instead - the right behavior for a costmap used for global planning.
  if (layered_costmap_ && layered_costmap_->isRolling())
  {
    std::lock_guard<std::mutex> lock(height_mutex_);
    std::fill(height_.begin(), height_.end(), std::numeric_limits<float>::quiet_NaN());
  }
}

void ElevationTraversabilityLayer::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!enabled_) return;
  auto node = node_.lock();
  if (!node) return;

  geometry_msgs::msg::TransformStamped transform;
  try
  {
    transform = tf_->lookupTransform(
        layered_costmap_->getGlobalFrameID(), msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(0.2));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 5000,
        "%s: TF %s->%s unavailable (%s)", name_.c_str(), layered_costmap_->getGlobalFrameID().c_str(),
        msg->header.frame_id.c_str(), ex.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  Eigen::Isometry3d eigen_tf = tf2::transformToEigen(transform);
  pcl::PointCloud<pcl::PointXYZ> cloud_global;
  pcl::transformPointCloud(cloud, cloud_global, eigen_tf.matrix().cast<float>());

  const double ox = transform.transform.translation.x;
  const double oy = transform.transform.translation.y;

  std::lock_guard<std::mutex> lock(height_mutex_);
  for (const auto& p : cloud_global.points)
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    const double range = std::hypot(p.x - ox, p.y - oy);
    if (range < obstacle_min_range_ || range > obstacle_max_range_) continue;
    unsigned int mx, my;
    if (!worldToMap(p.x, p.y, mx, my)) continue;
    const unsigned int idx = getIndex(mx, my);
    float& h = height_[idx];
    if (std::isnan(h))
    {
      h = p.z;
    }
    else if (p.z < h)
    {
      // Fast-fall: instantly adopt a lower reading - the true ground/ramp surface is
      // the lowest consistent return, and a single high outlier shouldn't get to
      // anchor the estimate.
      h += static_cast<float>(height_alpha_down_) * (p.z - h);
    }
    else
    {
      // Slow-rise: ~15s time constant at 10Hz, so a momentary high outlier (or a
      // person briefly crossing) can't poison the estimate, but a genuinely raised
      // surface eventually wins.
      h += static_cast<float>(height_alpha_up_) * (p.z - h);
    }
  }
  current_ = true;
}

double ElevationTraversabilityLayer::maxNeighborSlope(unsigned int mx, unsigned int my) const
{
  const int r = slope_check_radius_cells_;
  const int cx = static_cast<int>(mx);
  const int cy = static_cast<int>(my);
  const double res = getResolution();
  const double center_h = height_[getIndex(mx, my)];
  double max_slope = 0.0;
  bool any = false;

  static const int offsets[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  for (const auto& off : offsets)
  {
    const int nx = cx + off[0] * r;
    const int ny = cy + off[1] * r;
    if (nx < 0 || ny < 0 || nx >= static_cast<int>(getSizeInCellsX()) ||
        ny >= static_cast<int>(getSizeInCellsY()))
    {
      continue;
    }
    const unsigned int nidx = getIndex(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
    const float nh = height_[nidx];
    if (std::isnan(nh)) continue;
    const double dz = std::fabs(static_cast<double>(nh) - center_h);
    const double dist = res * std::hypot(off[0] * r, off[1] * r);
    if (dist < 1e-6) continue;
    max_slope = std::max(max_slope, std::atan2(dz, dist));
    any = true;
  }
  return any ? max_slope : std::numeric_limits<double>::quiet_NaN();
}

void ElevationTraversabilityLayer::updateBounds(
    double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double* min_x, double* min_y,
    double* max_x, double* max_y)
{
  if (!enabled_) return;

  // The framework does not reliably call this layer's own updateOrigin()/matchSize()
  // overrides to keep its plugin-owned grid in sync with the master costmap's
  // rolling window - confirmed empirically (this layer's own origin stayed frozen at
  // its onInitialize()-time value while the master costmap's origin correctly
  // rolled with the robot, verified via the debug_slope_grid topic reporting
  // origin (0,0) indefinitely while /local_costmap/costmap's own origin correctly
  // tracked the robot). Keep this layer's grid explicitly in sync every cycle
  // instead of assuming the framework does it - updateBounds() is the one hook
  // that's guaranteed to run every update, unlike updateOrigin()/matchSize().
  // Tolerance is half a cell, not a tight epsilon: confirmed empirically that
  // comparing with ~1e-9 caused a reset almost every single cycle (sub-cell
  // floating-point jitter in the master's rolling-window origin recomputation kept
  // reading as "different"), which wiped height_ before cloudCallback (10Hz) could
  // meaningfully repopulate it against updateCosts (5Hz for local_costmap) - the
  // debug_slope_grid stayed permanently empty as a result. A real window shift is
  // always at least a full resolution step; half a cell safely separates that from
  // float noise.
  const double origin_tol = getResolution() * 0.5;
  nav2_costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
  if (getSizeInCellsX() != master->getSizeInCellsX() || getSizeInCellsY() != master->getSizeInCellsY() ||
      std::abs(getResolution() - master->getResolution()) > 1e-9)
  {
    matchSize();
  }
  else if (std::abs(getOriginX() - master->getOriginX()) > origin_tol ||
           std::abs(getOriginY() - master->getOriginY()) > origin_tol)
  {
    updateOrigin(master->getOriginX(), master->getOriginY());
  }

  // Touch the whole grid every cycle rather than fine-grained dirty tracking - the
  // grid is small enough (<=3600 cells) that this is cheap, matching the project's
  // existing always_send_full_costmap intent elsewhere in the nav2 config.
  double wx, wy;
  mapToWorld(0, 0, wx, wy);
  *min_x = std::min(*min_x, wx);
  *min_y = std::min(*min_y, wy);
  mapToWorld(getSizeInCellsX() - 1, getSizeInCellsY() - 1, wx, wy);
  *max_x = std::max(*max_x, wx);
  *max_y = std::max(*max_y, wy);
}

void ElevationTraversabilityLayer::updateCosts(
    nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lock(height_mutex_);
  min_i = std::max(min_i, 0);
  min_j = std::max(min_j, 0);
  max_i = std::min(max_i, static_cast<int>(getSizeInCellsX()));
  max_j = std::min(max_j, static_cast<int>(getSizeInCellsY()));

  for (int j = min_j; j < max_j; ++j)
  {
    for (int i = min_i; i < max_i; ++i)
    {
      const unsigned int mx = static_cast<unsigned int>(i);
      const unsigned int my = static_cast<unsigned int>(j);
      const unsigned int idx = getIndex(mx, my);
      if (std::isnan(height_[idx]))
      {
        setCost(mx, my, nav2_costmap_2d::NO_INFORMATION);
        continue;
      }
      const double slope = maxNeighborSlope(mx, my);
      if (std::isnan(slope))
      {
        setCost(mx, my, nav2_costmap_2d::NO_INFORMATION);
        continue;
      }
      unsigned char cost;
      if (slope >= max_slope_rad_)
      {
        cost = nav2_costmap_2d::LETHAL_OBSTACLE;
      }
      else if (slope <= caution_slope_rad_)
      {
        cost = nav2_costmap_2d::FREE_SPACE;
      }
      else
      {
        const double t = (slope - caution_slope_rad_) / (max_slope_rad_ - caution_slope_rad_);
        cost = static_cast<unsigned char>(std::round(t * (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)));
      }
      setCost(mx, my, cost);
    }
  }

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);

  if (publish_debug_grid_)
  {
    publishDebugGrid();
  }
}

void ElevationTraversabilityLayer::publishDebugGrid()
{
  auto node = node_.lock();
  if (!node || !debug_pub_) return;

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = layered_costmap_->getGlobalFrameID();
  grid.header.stamp = node->now();
  grid.info.resolution = static_cast<float>(getResolution());
  grid.info.width = getSizeInCellsX();
  grid.info.height = getSizeInCellsY();
  grid.info.origin.position.x = getOriginX();
  grid.info.origin.position.y = getOriginY();
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<size_t>(grid.info.width) * grid.info.height, -1);
  for (unsigned int my = 0; my < getSizeInCellsY(); ++my)
  {
    for (unsigned int mx = 0; mx < getSizeInCellsX(); ++mx)
    {
      const unsigned int idx = getIndex(mx, my);
      if (std::isnan(height_[idx])) continue;
      const double slope = maxNeighborSlope(mx, my);
      if (std::isnan(slope)) continue;
      const int deg = static_cast<int>(std::round(slope * 180.0 / M_PI));
      grid.data[idx] = static_cast<int8_t>(std::min(deg, 100));
    }
  }
  debug_pub_->publish(grid);
}

void ElevationTraversabilityLayer::reset()
{
  std::lock_guard<std::mutex> lock(height_mutex_);
  std::fill(height_.begin(), height_.end(), std::numeric_limits<float>::quiet_NaN());
}

}  // namespace lio_sam_bringup

PLUGINLIB_EXPORT_CLASS(lio_sam_bringup::ElevationTraversabilityLayer, nav2_costmap_2d::Layer)
