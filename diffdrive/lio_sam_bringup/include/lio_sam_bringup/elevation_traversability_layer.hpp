#ifndef LIO_SAM_BRINGUP__ELEVATION_TRAVERSABILITY_LAYER_HPP_
#define LIO_SAM_BRINGUP__ELEVATION_TRAVERSABILITY_LAYER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace lio_sam_bringup
{

// Marks costmap cells lethal based on *local slope* rather than absolute point
// height, so a gentle ramp reads as free/low-cost while an actual wall (near-90
// degree local slope regardless of its exact height) still reads lethal. Replaces
// nav2_costmap_2d::ObstacleLayer/VoxelLayer, which only look at
// max_obstacle_height - see lio_sam_bringup's README for the ramp-climbing bug
// this fixes and why coexisting with those layers (rather than replacing them)
// wouldn't work (max-combination lets their lethal marks win regardless of this
// layer's lower cost).
class ElevationTraversabilityLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  ElevationTraversabilityLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override { return true; }
  void matchSize() override;
  void updateOrigin(double new_origin_x, double new_origin_y) override;

private:
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  // Max local slope (radians) from cell (mx,my) to any neighbor within
  // slope_check_radius_cells_ that has data. Returns NaN if too few
  // data-bearing neighbors exist to judge.
  double maxNeighborSlope(unsigned int mx, unsigned int my) const;
  void publishDebugGrid();

  std::vector<float> height_;
  std::mutex height_mutex_;

  std::string point_cloud_topic_;
  double obstacle_min_range_, obstacle_max_range_;
  int slope_check_radius_cells_;
  double caution_slope_rad_, max_slope_rad_;
  double height_alpha_down_, height_alpha_up_;
  bool publish_debug_grid_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_pub_;
};

}  // namespace lio_sam_bringup

#endif  // LIO_SAM_BRINGUP__ELEVATION_TRAVERSABILITY_LAYER_HPP_
