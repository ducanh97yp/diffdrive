#!/usr/bin/env python3
"""Republish the monitoring EKF's smoothed position as a NavSatFix.

Subscribes to the monitoring EKF's output Odometry (map-frame x/y, smoothed,
no discrete GPSFactor-correction jumps) and converts it back to lat/lon via
navsat_transform_node's /toLL service, publishing sensor_msgs/msg/NavSatFix on
/monitoring/gps_fix - a smooth position signal for AerialMap or any external
GIS/dashboard tool, instead of driving those from raw (noisier, and prone to
the same discrete jumps as LIO-SAM's own pose graph) GPS/odometry sources.

Requires navsat_transform_node's /toLL service to be running (i.e. this is
meant to run alongside lio_sam_gps_outdoor_launch.py, not standalone).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from robot_localization.srv import ToLL


class GpsFixFromOdom(Node):
    def __init__(self):
        super().__init__('gps_fix_from_odom')
        self.client = self.create_client(ToLL, '/toLL')
        self.pub = self.create_publisher(NavSatFix, '/monitoring/gps_fix', 10)
        self.sub = self.create_subscription(
            Odometry, '/monitoring/global_odometry', self.odom_cb, qos_profile_sensor_data)
        self._busy = False

    def odom_cb(self, msg: Odometry):
        if self._busy or not self.client.service_is_ready():
            return
        self._busy = True
        request = ToLL.Request()
        request.map_point = msg.pose.pose.position
        future = self.client.call_async(request)
        future.add_done_callback(lambda f: self._on_toll_result(f, msg))

    def _on_toll_result(self, future, odom_msg: Odometry):
        self._busy = False
        result = future.result()
        if result is None:
            return
        fix = NavSatFix()
        fix.header.stamp = odom_msg.header.stamp
        fix.header.frame_id = 'gps_link'
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = result.ll_point.latitude
        fix.longitude = result.ll_point.longitude
        fix.altitude = result.ll_point.altitude
        self.pub.publish(fix)


def main():
    rclpy.init()
    node = GpsFixFromOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
