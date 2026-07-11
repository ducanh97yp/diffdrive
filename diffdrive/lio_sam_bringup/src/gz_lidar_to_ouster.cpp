// Adds LIO-SAM's OusterPointXYZIRT layout on top of Gazebo's simulated lidar cloud
// (x,y,z,intensity,ring - confirmed via `ros2 topic echo /points`). Only x,y,z,
// intensity,t,ring are actually consumed by LIO-SAM's imageProjection.cpp
// (cachePointCloud()'s SensorType::OUSTER branch, confirmed by reading the ros2
// branch source directly) - reflectivity/noise/range are populated anyway with
// physically-derived values (not left at zero) so a real ouster-ros bring-up later
// sees a structurally familiar message, even though LIO-SAM itself ignores them.
// t is left at 0 for every point, same reasoning as the earlier Velodyne adapter had:
// Gazebo's gpu_lidar publishes an instantaneous full sweep with no rolling shutter,
// so there is no real per-point scan time to simulate - an all-zero "t" field is the
// physically correct representation here, not a shortcut.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace gz_lidar
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace gz_lidar
POINT_CLOUD_REGISTER_POINT_STRUCT(gz_lidar::Point,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                       std::uint16_t, ring, ring))

namespace ouster_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint32_t t;
  std::uint16_t reflectivity;
  std::uint8_t ring;
  std::uint16_t noise;
  std::uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                       std::uint32_t, t, t)(std::uint16_t, reflectivity, reflectivity)(
                                       std::uint8_t, ring, ring)(std::uint16_t, noise, noise)(
                                       std::uint32_t, range, range))

class GzLidarToOuster : public rclcpp::Node
{
public:
  GzLidarToOuster() : Node("gz_lidar_to_ouster")
  {
    // Axis-aligned self-filter box, in laser_frame coordinates. OS1's FOV is
    // symmetric (+-22.5deg) and the mount is upright (rpy 0 0 0, see lidar.xacro -
    // no downward-flip trick needed, unlike the old Mid-360 mount). Bounds below are
    // empirically derived (mount height 0.15) via direct point-cloud inspection of
    // raw /points: a self-hit cluster was found at range 0.24-0.50m, z tightly
    // banded to -0.117..-0.078 (consistent with grazing a single flat surface - the
    // chassis top - at roughly constant depth below the sensor across many
    // azimuths), with a clear gap before legitimate environment returns resume at
    // range >0.6m (point count drops 520/312/152/54 across 0-0.3/0.3-0.4/0.4-0.5/
    // 0.5-0.6m bins, then jumps back up to 1146 for 0.6-0.8m). Bounds below are the
    // observed x/y/z extent of that cluster plus a 3cm margin.
    self_filter_min_x_ = declare_parameter("self_filter_min_x", -0.24);
    self_filter_max_x_ = declare_parameter("self_filter_max_x", 0.51);
    self_filter_min_y_ = declare_parameter("self_filter_min_y", -0.31);
    self_filter_max_y_ = declare_parameter("self_filter_max_y", 0.31);
    self_filter_min_z_ = declare_parameter("self_filter_min_z", -0.15);
    self_filter_max_z_ = declare_parameter("self_filter_max_z", -0.05);

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("ouster_points", rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "points", rclcpp::SensorDataQoS(),
        std::bind(&GzLidarToOuster::callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to points, publishing ouster_points");
  }

private:
  bool inSelfFilterBox(float x, float y, float z) const
  {
    return x >= self_filter_min_x_ && x <= self_filter_max_x_ && y >= self_filter_min_y_ &&
           y <= self_filter_max_y_ && z >= self_filter_min_z_ && z <= self_filter_max_z_;
  }

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<gz_lidar::Point> in;
    pcl::fromROSMsg(*msg, in);

    pcl::PointCloud<ouster_ros::Point> out;
    out.reserve(in.size());
    for (const auto& p : in.points)
    {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      {
        continue;
      }
      if (inSelfFilterBox(p.x, p.y, p.z))
      {
        continue;
      }
      ouster_ros::Point op;
      op.x = p.x;
      op.y = p.y;
      op.z = p.z;
      op.intensity = p.intensity;
      op.ring = static_cast<std::uint8_t>(std::min<std::uint16_t>(p.ring, 255));
      op.t = 0;
      op.reflectivity = static_cast<std::uint16_t>(std::clamp(p.intensity, 0.0f, 65535.0f));
      op.noise = 0;
      float range_m = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
      op.range = static_cast<std::uint32_t>(range_m * 1000.0f);
      out.push_back(op);
    }

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(out, out_msg);
    out_msg.header = msg->header;
    pub_->publish(out_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  double self_filter_min_x_, self_filter_max_x_;
  double self_filter_min_y_, self_filter_max_y_;
  double self_filter_min_z_, self_filter_max_z_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzLidarToOuster>());
  rclcpp::shutdown();
  return 0;
}
