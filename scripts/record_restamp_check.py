#!/usr/bin/env python3
"""Records the raw IMU and the restamped output side by side, for plot_restamp_check.py.

    ros2 run imu_restamper record_restamp_check.py 30 check.csv [--raw /imu] [--out /imu/restamped] [--cloud TOPIC]

Rows: "<R|O>,<stamp_ns>,<arrival_ns>,ax,ay,az,gx,gy,gz" (R = raw input, O = restamped output). The
arrival time is taken in this process, so arrival(O) - arrival(R) is the delay the node adds.
--cloud also subscribes to a point cloud (serialized, never decoded), as the SLAM will: some drivers
stall the IMU only while they deliver scans to a subscriber.
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2


def stamp_ns(m):
    return m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec


class Recorder(Node):
    def __init__(self, raw_topic, out_topic, cloud_topic):
        super().__init__('restamp_check_recorder')
        self.rows = []
        self.clouds = 0
        qos = QoSProfile(depth=500, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Imu, raw_topic, lambda m: self.add('R', m), qos)
        self.create_subscription(Imu, out_topic, lambda m: self.add('O', m), qos)
        if cloud_topic:
            self.create_subscription(
                PointCloud2, cloud_topic, self.on_cloud,
                QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE), raw=True)

    def add(self, tag, m):
        arrival = time.time_ns()
        a, w = m.linear_acceleration, m.angular_velocity
        self.rows.append(
            f'{tag},{stamp_ns(m)},{arrival},{a.x!r},{a.y!r},{a.z!r},{w.x!r},{w.y!r},{w.z!r}')

    def on_cloud(self, _):
        self.clouds += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('duration', type=float)
    ap.add_argument('output')
    ap.add_argument('--raw', default='/imu')
    ap.add_argument('--out', default='/imu/restamped')
    ap.add_argument('--cloud', help='also subscribe to this point cloud topic')
    args = ap.parse_args()
    rclpy.init()
    node = Recorder(args.raw, args.out, args.cloud)
    end = time.time() + args.duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
    with open(args.output, 'w') as f:
        f.write('\n'.join(node.rows) + '\n')
    if args.cloud:
        print(f'cloud messages received: {node.clouds}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
