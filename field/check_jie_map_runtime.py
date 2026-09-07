#!/usr/bin/env python3
"""Run isolated converter, marker and JIE planning checks without driving a robot.

Source the JIE workspace first. This command owns only its child processes and
uses a separate ROS domain; it never starts a controller or publishes velocity.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker

from verify_jie_tunnel_pcd import CORRIDORS, read_binary_xyz_pcd


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pcd', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--resolution', type=float, default=.04)
    parser.add_argument('--domain', type=int, default=173)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1',
                      RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
    os.environ['CYCLONEDDS_URI'] = ('<CycloneDDS><Domain Id="any"><Discovery>'
        '<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100'
        '</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>')
    points = read_binary_xyz_pcd(args.pcd).astype(float)
    expected = np.unique(np.floor(points / args.resolution).astype(np.int32), axis=0)
    processes, logs = [], []
    report = {'pcd': str(args.pcd.resolve()), 'resolution': args.resolution}
    rclpy.init()
    node = rclpy.create_node('jie_runtime_map_audit')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    received = {}
    _subscriptions = [
        node.create_subscription(Marker, '/octomap_occupied_markers',
                                 lambda m: received.update(marker=m), qos),
        node.create_subscription(PointCloud2, '/risk_cost_cells',
                                 lambda m: received.update(risk=m), qos),
        node.create_subscription(PathMsg, '/planned_path',
                                 lambda m: received.update(path=m), qos),
    ]
    start_pub = node.create_publisher(PointStamped, '/start_point', qos)
    goal_pub = node.create_publisher(PointStamped, '/goal_point', qos)

    def start(package, executable, params):
        binary = Path(get_package_prefix(package)) / 'lib' / package / executable
        command = [str(binary), '--ros-args']
        for k, v in params.items():
            command += ['-p', f'{k}:={str(v).lower() if isinstance(v, bool) else v}']
        log = (args.output / (executable + '.log')).open('w')
        logs.append(log)
        processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))

    def wait(predicate, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.2)
            if predicate():
                return
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('a test node exited; inspect logs')
        raise TimeoutError('map/runtime check timed out; inspect logs')

    def publish(pub, xyz):
        m = PointStamped()
        m.header.frame_id = 'map'
        m.header.stamp = node.get_clock().now().to_msg()
        m.point.x, m.point.y, m.point.z = xyz
        pub.publish(m)

    try:
        started = time.monotonic()
        start('octo_planner', 'jie_path_node', {
            'robot_radius': .25, 'robot_radius_xy': .28, 'robot_height': .225,
            'max_iterations': 500000, 'snap_search_radius_cells': 12,
            'require_ground_support': True, 'strict_direct_ground_support': False,
            'ground_support_xy_radius_cells': 1, 'ground_support_depth_cells': 1,
            'preblocked_costmap_radius_cells': 3, 'preblocked_costmap_weight': 2.5,
        })
        start('jie_octomap', 'octomap_to_occupied_markers_node', {})
        start('jie_octomap', 'pcd_to_octomap_node', {
            'pcd_file': str(args.pcd.resolve()), 'resolution': args.resolution,
            'min_points_per_voxel': 1, 'min_cluster_voxels': 1,
        })
        wait(lambda: 'marker' in received, 60)
        marker = received['marker']
        actual = np.array([(p.x, p.y, p.z) for p in marker.points])
        actual_keys = np.unique(np.floor(actual / args.resolution).astype(np.int32), axis=0)
        np.testing.assert_array_equal(actual_keys, expected)
        assert len(actual) == len(expected)
        report['marker_cells_match_pcd'] = len(actual)
        print('Marker geometry matches PCD:', len(actual), flush=True)
        wait(lambda: 'risk' in received, 1200)
        report['map_ready_seconds'] = time.monotonic() - started
        print('Map ready:', report['map_ready_seconds'], flush=True)
        report['paths'] = []
        for corridor in CORRIDORS:
            received.pop('path', None)
            publish(start_pub, corridor['start'])
            publish(goal_pub, corridor['goal'])
            def at_goal():
                path = received.get('path')
                if path is None or not path.poses:
                    return False
                p = path.poses[-1].pose.position
                return np.linalg.norm(np.array([p.x, p.y]) - corridor['goal'][:2]) < .15
            wait(at_goal, 60)
            path = received['path']
            z = [p.pose.position.z for p in path.poses]
            assert max(z) < .15, 'path incorrectly used the roof'
            report['paths'].append({'name': corridor['name'], 'poses': len(z),
                                    'z_range': [min(z), max(z)]})
        report['passed'] = True
        print(json.dumps(report, indent=2), flush=True)
    finally:
        for p in processes:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        for p in processes:
            try:
                p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
        for log in logs:
            log.close()
        (args.output / 'runtime_report.json').write_text(json.dumps(report, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
