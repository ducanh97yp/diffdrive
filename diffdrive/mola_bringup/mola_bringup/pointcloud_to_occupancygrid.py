#!/usr/bin/env python3
import re
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros

_ICP_QUALITY_RE = re.compile(r"icp_quality:\s*([\d\.]+)")


def quat_translation_to_matrix(t):
    """TransformStamped -> (3x3 rotation, 3 translation), for a one-shot vectorized
    transform of the whole point cloud instead of a per-point tf2 lookup/transform."""
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    p = t.transform.translation
    return r, np.array([p.x, p.y, p.z])


class PointCloudToOccupancyGrid(Node):
    """Builds a 2D nav_msgs/OccupancyGrid by height-slicing the live 3D lidar point
    cloud and accumulating hits in map frame while driving around (in mola_localize's
    'map' frame). Cells are only ever marked occupied on a hit; everything else stays
    unknown (-1) - Nav2's live obstacle-layer (fed by the same point cloud) covers
    real-time safety, so this is only meant to give the global planner known walls,
    not a fully free-space-cleared map. Save the result with the standard
    `ros2 run nav2_map_server map_saver_cli`.

    Deliberately fully vectorized with numpy (transform+bin the whole cloud in one
    shot) instead of a per-point Python loop - at 5Hz x ~8000 pts/scan a per-point loop
    here would repeat the exact CPU-blowup class of bug just fixed in
    plot_lidar_trajectory.py.
    """

    def __init__(self):
        super().__init__('pointcloud_to_occupancygrid')

        self.declare_parameter('input_topic', '/livox/lidar_filtered')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('width_m', 40.0)
        self.declare_parameter('height_m', 40.0)
        self.declare_parameter('origin_x', -20.0)
        self.declare_parameter('origin_y', -20.0)
        # Height band in map frame (assumed gravity-aligned, floor ~ z=0): excludes the
        # floor plane and anything above the robot, keeping wall/furniture returns.
        self.declare_parameter('min_height', 0.05)
        self.declare_parameter('max_height', 0.6)
        # A cell needs this many separate hits before being marked occupied, to ignore
        # one-off noisy returns.
        self.declare_parameter('occ_hit_threshold', 2)
        self.declare_parameter('publish_rate_hz', 1.0)
        # Below this, MOLA's own diagnostics say ICP tracking is unreliable (lost/
        # relocalizing) - the map->lidar transform can be badly wrong then, so scans
        # get dropped instead of smearing the same wall across multiple drifted poses
        # (this is what produced the fan-of-duplicate-walls artifact in earlier maps).
        self.declare_parameter('min_icp_quality', 0.7)

        input_topic = self.get_parameter('input_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.resolution = float(self.get_parameter('resolution').value)
        self.width_m = float(self.get_parameter('width_m').value)
        self.height_m = float(self.get_parameter('height_m').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)
        self.occ_hit_threshold = int(self.get_parameter('occ_hit_threshold').value)
        self.min_icp_quality = float(self.get_parameter('min_icp_quality').value)
        # None means "no reading yet" - stay permissive rather than blocking the map
        # before any diagnostics message has arrived.
        self.icp_quality = None

        self.width_cells = max(1, int(round(self.width_m / self.resolution)))
        self.height_cells = max(1, int(round(self.height_m / self.resolution)))
        self.hit_count = np.zeros((self.height_cells, self.width_cells), dtype=np.int32)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(PointCloud2, input_topic, self._cloud_callback, 10)
        self.create_subscription(
            String, '/mola_diagnostics/lidar_odom/status', self._status_callback, 10)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)

        self.create_service(Trigger, 'clear_map', self._clear_map_cb)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._publish_map)

        self.get_logger().info(
            f"Subscribed to: {input_topic}, building {self.width_cells}x{self.height_cells} "
            f"grid @ {self.resolution} m/cell in frame '{self.map_frame}'. "
            f"Height band kept: [{self.min_height}, {self.max_height}] m."
        )

    def _status_callback(self, msg: String):
        match = _ICP_QUALITY_RE.search(msg.data)
        if match:
            self.icp_quality = float(match.group(1))

    def _cloud_callback(self, msg: PointCloud2):
        if self.icp_quality is not None and self.icp_quality < self.min_icp_quality:
            self.get_logger().warn(
                f"Skipping scan: icp_quality={self.icp_quality:.2f} < "
                f"{self.min_icp_quality} (tracking unreliable, map->lidar transform "
                f"can't be trusted right now)",
                throttle_duration_sec=2.0)
            return
        try:
            # Deliberately NOT looking up the transform at msg.header.stamp with a
            # blocking timeout: this callback runs on the same single-threaded
            # executor as tf2_ros.TransformListener's own /tf subscription, so a
            # multi-second blocking wait here starves that same subscription of the
            # chance to process the very /tf messages it's waiting on - a classic
            # single-threaded-executor self-stall, not a MOLA lag issue (confirmed:
            # MOLA's own average_process_time/icp_quality stayed healthy throughout,
            # only this node's lookups kept timing out). Using the latest available
            # transform instead accepts a little position error (up to ~1 map
            # update's worth of robot motion) but never blocks.
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id, Time())
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"No transform {msg.header.frame_id}->{self.map_frame}: {ex}",
                                    throttle_duration_sec=5.0)
            return

        points = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        # skip_nans only guards the structured-array view read_points_numpy builds;
        # gz's simulated lidar can still emit non-finite returns (max-range rays) that
        # slip through, which then poison the matmul below with NaN/Inf.
        points = points[np.isfinite(points).all(axis=1)]
        if points.shape[0] == 0:
            return

        r, t = quat_translation_to_matrix(tf)
        points_map = points @ r.T + t

        z = points_map[:, 2]
        keep = (z >= self.min_height) & (z <= self.max_height)
        if not np.any(keep):
            return
        xy = points_map[keep, :2]

        cols = np.floor((xy[:, 0] - self.origin_x) / self.resolution).astype(np.int64)
        rows = np.floor((xy[:, 1] - self.origin_y) / self.resolution).astype(np.int64)
        valid = (cols >= 0) & (cols < self.width_cells) & (rows >= 0) & (rows < self.height_cells)
        if not np.any(valid):
            return

        np.add.at(self.hit_count, (rows[valid], cols[valid]), 1)

    def _publish_map(self):
        grid = np.full(self.hit_count.shape, -1, dtype=np.int8)
        grid[self.hit_count >= self.occ_hit_threshold] = 100

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.resolution
        msg.info.width = self.width_cells
        msg.info.height = self.height_cells
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()
        self.map_pub.publish(msg)

    def _clear_map_cb(self, request, response):
        self.hit_count[:] = 0
        response.success = True
        response.message = "Occupancy grid cleared."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToOccupancyGrid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
