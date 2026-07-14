#!/usr/bin/env python3
"""Write a GIS World File (.pgw) for a saved map_saver_cli PGM/YAML pair.

Turns a saved 2D map (`ros2 run nav2_map_server map_saver_cli -f <name>`,
run against pcd_map_localizer's /map topic) into a georeferenced raster
loadable directly in QGIS or similar: reads the map's origin (in the SLAM
`map` frame), converts it to lat/lon via navsat_transform_node's /toLL
service, projects to UTM (offline, via the `cs2cs` CLI - no extra packages
needed), and writes the standard 6-parameter World File next to the PGM.

World files need a *projected* (meters) CRS, not raw lat/lon degrees, or
east-west distances distort away from the equator - UTM is the natural
choice since the map's resolution is already in meters/cell.

Requires navsat_transform_node's /toLL service to be running (i.e. run this
against a live lio_sam_gps_outdoor_launch.py, not standalone).

Usage:
    ros2 run lio_sam_bringup georeference_map.py --map my_map.yaml
"""
import argparse
import math
import subprocess
import sys

import rclpy
import yaml
from geometry_msgs.msg import Point
from PIL import Image
from rclpy.node import Node
from robot_localization.srv import ToLL


class Georeferencer(Node):
    def __init__(self):
        super().__init__('georeference_map')
        self.client = self.create_client(ToLL, '/toLL')

    def to_ll(self, x, y):
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                '/toLL service not available - is navsat_transform_node running? '
                'Launch the outdoor GPS stack first (e.g. lio_sam_gps_outdoor_launch.py).')
        request = ToLL.Request()
        request.map_point = Point(x=x, y=y, z=0.0)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().ll_point


def utm_zone(lon):
    return int((lon + 180) / 6) + 1


def lonlat_to_utm(lon, lat):
    zone = utm_zone(lon)
    epsg = (32600 if lat >= 0 else 32700) + zone
    # cs2cs on this system's PROJ version (9.4.0) respects EPSG:4326's official
    # axis order (lat, lon) rather than the historical lon/lat convention -
    # confirmed empirically: "33.83 -84.42" matches navsat_transform_node's own
    # internal UTM computation almost exactly, "-84.42 33.83" does not.
    result = subprocess.run(
        ['cs2cs', 'EPSG:4326', f'EPSG:{epsg}', '-f', '%.4f'],
        input=f'{lat} {lon}\n', capture_output=True, text=True, check=True)
    parts = result.stdout.split()
    easting, northing = float(parts[0]), float(parts[1])
    return easting, northing, epsg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True, help='map_saver_cli output .yaml file')
    args = parser.parse_args()

    with open(args.map) as f:
        map_yaml = yaml.safe_load(f)

    resolution = map_yaml['resolution']
    origin_x, origin_y, origin_yaw = map_yaml['origin']
    if abs(origin_yaw) > 1e-6:
        print(f'WARNING: map origin yaw ({origin_yaw} rad) is non-zero - this script '
              'only writes an axis-aligned world file, the output will be misrotated.',
              file=sys.stderr)

    image_path = map_yaml['image']
    if not image_path.startswith('/'):
        import os
        image_path = os.path.join(os.path.dirname(os.path.abspath(args.map)), image_path)
    with Image.open(image_path) as img:
        width, height = img.size

    rclpy.init()
    node = Georeferencer()
    try:
        ll = node.to_ll(origin_x, origin_y)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    easting, northing, epsg = lonlat_to_utm(ll.longitude, ll.latitude)

    # map_saver_cli's origin is the *bottom-left* corner of the bottom-left pixel;
    # world files need the *top-left* pixel *center*.
    top_left_x = easting + resolution / 2.0
    top_left_y = northing + height * resolution - resolution / 2.0

    pgw_path = image_path.rsplit('.', 1)[0] + '.pgw'
    with open(pgw_path, 'w') as f:
        f.write(f'{resolution}\n0.0\n0.0\n{-resolution}\n{top_left_x}\n{top_left_y}\n')

    print(f'Wrote {pgw_path}')
    print(f'Origin lat/lon: {ll.latitude}, {ll.longitude}')
    print(f'CRS: EPSG:{epsg} (UTM zone {utm_zone(ll.longitude)})')
    print(f'In QGIS: load {image_path}, then Layer Properties -> Assign CRS -> EPSG:{epsg}')


if __name__ == '__main__':
    main()
