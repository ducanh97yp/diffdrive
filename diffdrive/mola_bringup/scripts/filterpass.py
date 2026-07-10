#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

# --- We no longer need these ---
# from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

try:
    from sensor_msgs_py import point_cloud2 as pc2
except Exception as e:
    raise RuntimeError(
        "sensor_msgs_py.point_cloud2 is required. Install 'ros-<distro>-sensor-msgs-py'."
    ) from e


class LidarCombinedFilter(Node):
    def __init__(self):
        super().__init__('lidar_combined_filter')
        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/lidar_filtered')
        self.declare_parameter('angle_deg', 360.0)
        self.declare_parameter('keep_within', True)
        self.declare_parameter('min_intensity', 1.0)
        self.declare_parameter('max_intensity', 255.0)
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.angle_rad = math.radians(self.get_parameter('angle_deg').get_parameter_value().double_value)
        self.keep_within = self.get_parameter('keep_within').get_parameter_value().bool_value
        self.min_intensity = self.get_parameter('min_intensity').get_parameter_value().double_value
        self.max_intensity = self.get_parameter('max_intensity').get_parameter_value().double_value
        self.sub = self.create_subscription(PointCloud2, input_topic, self.pc_callback, 10)
    
        self.pub = self.create_publisher(PointCloud2, output_topic, 10)

        self.get_logger().info(
            f"Subscribed to: {input_topic}\n"
            f"Publishing to: {output_topic}\n"
            f"Angle: ±{math.degrees(self.angle_rad):.1f}°, keep_within={self.keep_within}\n"
            f"Intensity range: {self.min_intensity}–{self.max_intensity}\n"
            f"QoS: System Default (Auto-negotiating)"
        )

    def pc_callback(self, msg: PointCloud2):
        field_names = [f.name for f in msg.fields]
        if 'x' not in field_names or 'y' not in field_names:
            self.get_logger().warn("No 'x' or 'y' field — republishing original.")
            self.pub.publish(msg)
            return

        ix = field_names.index('x')
        iy = field_names.index('y')
        idx_intensity = field_names.index('intensity') if 'intensity' in field_names else None

        selected_points = []
        total_count = 0
        keep_count = 0

        for p in pc2.read_points(msg, field_names=field_names, skip_nans=True):
            total_count += 1
            try:
                x = float(p[ix])
                y = float(p[iy])
            except Exception:
                continue

            az = math.atan2(y, x)
            inside_angle = abs(az) <= self.angle_rad
            angle_ok = inside_angle if self.keep_within else not inside_angle

            intensity_ok = True
            if idx_intensity is not None:
                try:
                    intensity = float(p[idx_intensity])
                    intensity_ok = self.min_intensity <= intensity <= self.max_intensity
                except Exception:
                    pass

            if angle_ok and intensity_ok:
                selected_points.append(p)
                keep_count += 1

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id

        new_msg = pc2.create_cloud(header, msg.fields, selected_points)
        new_msg.height = 1
        new_msg.width = len(selected_points)
        new_msg.is_dense = bool(selected_points)

        self.pub.publish(new_msg)

        if total_count > 0 and keep_count == 0:
            self.get_logger().warn("No points passed filter!")
        elif total_count > 0 and total_count % 5000 == 0:
            self.get_logger().info(f"Filtered {keep_count}/{total_count} points kept.")

def main(args=None):
    rclpy.init(args=args)
    node = LidarCombinedFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()