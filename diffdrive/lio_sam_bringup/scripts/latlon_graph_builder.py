#!/usr/bin/env python3
"""Convert a lat/lon waypoint list into a nav2_route GeoJSON graph.

Input YAML format (nodes with directed edges - list a neighbor in both
directions for a bidirectional path, matching nav2_route's own one-way
edge convention):

    nodes:
      - id: 0
        lat: 21.028511
        lon: 105.804817
        edges: [1]
      - id: 1
        lat: 21.028600
        lon: 105.804900
        edges: [0, 2]
      - id: 2
        lat: 21.028700
        lon: 105.804950
        edges: [1]

Requires navsat_transform_node's /fromLL service to be running (i.e. an
outdoor GPS launch, e.g. lio_sam_gps_outdoor_launch.py, already up) - each
node's lat/lon is converted to the live map-frame origin at call time, so
run this against the same launch/world the graph will actually be used
with.

Usage:
    ros2 run lio_sam_bringup latlon_graph_builder.py \\
        --input campus_waypoints.yaml --output campus_graph.geojson
"""
import argparse
import json

import rclpy
import yaml
from geographic_msgs.msg import GeoPoint
from rclpy.node import Node
from robot_localization.srv import FromLL


class LatLonGraphBuilder(Node):
    def __init__(self):
        super().__init__('latlon_graph_builder')
        self.client = self.create_client(FromLL, '/fromLL')

    def from_ll(self, lat, lon):
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                '/fromLL service not available - is navsat_transform_node running? '
                'Launch the outdoor GPS stack first (e.g. lio_sam_gps_outdoor_launch.py).')
        request = FromLL.Request()
        request.ll_point = GeoPoint(latitude=lat, longitude=lon, altitude=0.0)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map_point


def build_graph(nodes, builder: LatLonGraphBuilder):
    features = []
    for n in nodes:
        map_point = builder.from_ll(n['lat'], n['lon'])
        features.append({
            'type': 'Feature',
            'properties': {'id': n['id'], 'frame': 'map'},
            'geometry': {'type': 'Point', 'coordinates': [map_point.x, map_point.y]},
        })

    by_id = {n['id']: n for n in nodes}
    edge_id = 1000  # start well above node ids to avoid accidental collisions
    for n in nodes:
        start = by_id[n['id']]
        for neighbor_id in n.get('edges', []):
            end = by_id[neighbor_id]
            start_pt = next(f for f in features if f['properties']['id'] == start['id'])
            end_pt = next(f for f in features if f['properties']['id'] == end['id'])
            features.append({
                'type': 'Feature',
                'properties': {'id': edge_id, 'startid': start['id'], 'endid': end['id']},
                'geometry': {
                    'type': 'MultiLineString',
                    'coordinates': [[start_pt['geometry']['coordinates'],
                                      end_pt['geometry']['coordinates']]],
                },
            })
            edge_id += 1

    return {
        'type': 'FeatureCollection',
        'name': 'graph',
        'crs': {'type': 'name', 'properties': {'name': 'urn:ogc:def:crs:EPSG::3857'}},
        'features': features,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, help='Input lat/lon waypoint YAML file')
    parser.add_argument('--output', required=True, help='Output nav2_route GeoJSON graph file')
    args = parser.parse_args()

    with open(args.input) as f:
        nodes = yaml.safe_load(f)['nodes']

    rclpy.init()
    builder = LatLonGraphBuilder()
    try:
        graph = build_graph(nodes, builder)
    finally:
        builder.destroy_node()
        rclpy.shutdown()

    with open(args.output, 'w') as f:
        json.dump(graph, f, indent=4)

    print(f'Wrote {len(nodes)} nodes to {args.output}')


if __name__ == '__main__':
    main()
