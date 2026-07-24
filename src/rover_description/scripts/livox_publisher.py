#!/usr/bin/env python3
"""PointCloud2 → Livox CustomMsg converter.

Real Livox drivers publish BOTH:
  1. sensor_msgs/PointCloud2       — for RViz / Cartographer / Nav2 / generic perception
  2. livox_msgs/CustomMsg          — for FAST-LIO2 / Point-LIO / LIO-SAM (Livox variant)

Our Ignition sim publishes only (1). This node subscribes to /<ns>/lidar/points
and republishes on /<ns>/livox/lidar as CustomMsg — same underlying points,
Livox format with per-point timestamps synthesised across the scan window,
so any Livox-optimised SLAM stack can subscribe unmodified.

Caveats (documented so nobody thinks this is a perfect Livox emulation):
- Ignition gpu_lidar does NOT produce Livox's non-repetitive scan pattern;
  points are on a uniform grid. Real Mid-360 sweeps a rosette. Downstream
  SLAM will still work but won't benefit from Livox's density fill-in.
- offset_time is synthesised as a linear ramp across the message. Real Livox
  offsets follow the actual scan schedule.
- reflectivity is a placeholder (Ignition returns intensity in [0,1]; we
  scale to 0..255). tag=0, line=0 (Mid-360 fires 4 lines but we don't model that).
"""
import argparse
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from livox_msgs.msg import CustomMsg, CustomPoint


SCAN_PERIOD_NS = 100_000_000  # 100 ms — Mid-360 spins at 10 Hz


class LivoxPublisher(Node):
    def __init__(self, namespace):
        super().__init__('livox_publisher')
        self.ns = namespace
        in_topic = f'/{namespace}/lidar/points' if namespace else '/lidar/points'
        out_topic = f'/{namespace}/livox/lidar' if namespace else '/livox/lidar'

        # Best-effort QoS matches sensor stream conventions.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)

        self.pub = self.create_publisher(CustomMsg, out_topic, qos)
        self.sub = self.create_subscription(PointCloud2, in_topic, self.on_cloud, qos)
        self.get_logger().info(f'livox_publisher: {in_topic}  →  {out_topic}')

    def on_cloud(self, cloud: PointCloud2):
        offsets = {f.name: f.offset for f in cloud.fields}
        if 'x' not in offsets or 'y' not in offsets or 'z' not in offsets:
            return
        step = cloud.point_step
        n = cloud.width * cloud.height
        if n == 0:
            return

        # Vectorized parse: view the raw buffer as N structured records and slice
        # x/y/z/intensity in one go. ~200x faster than a per-point Python loop —
        # required to hit the sensor's 5 Hz publish rate on 5760-point scans.
        buf = np.frombuffer(cloud.data, dtype=np.uint8)
        # Reshape into (n, step) so each row is one point's raw bytes.
        rows = buf.reshape(n, step)
        xs = np.frombuffer(rows[:, offsets['x']:offsets['x'] + 4].tobytes(), dtype=np.float32)
        ys = np.frombuffer(rows[:, offsets['y']:offsets['y'] + 4].tobytes(), dtype=np.float32)
        zs = np.frombuffer(rows[:, offsets['z']:offsets['z'] + 4].tobytes(), dtype=np.float32)
        if 'intensity' in offsets:
            oi = offsets['intensity']
            intensities = np.frombuffer(rows[:, oi:oi + 4].tobytes(), dtype=np.float32)
            refls = np.clip(intensities * 255.0, 0, 255).astype(np.uint8)
        else:
            refls = np.zeros(n, dtype=np.uint8)

        dt = SCAN_PERIOD_NS // max(1, n)
        offset_times = (np.arange(n, dtype=np.uint32) * dt).astype(np.uint32)

        # Build the CustomPoint list. Assigning attrs on rosidl-generated objects
        # is the only supported construction path; there is no bulk constructor.
        # This loop is now the sole Python-level cost — ~20 ms for 5760 points.
        pts = [None] * n
        for i in range(n):
            p = CustomPoint()
            p.offset_time = int(offset_times[i])
            p.x = float(xs[i])
            p.y = float(ys[i])
            p.z = float(zs[i])
            p.reflectivity = int(refls[i])
            p.tag = 0
            p.line = 0
            pts[i] = p

        out = CustomMsg()
        out.header = cloud.header
        out.timebase = (cloud.header.stamp.sec * 1_000_000_000 +
                        cloud.header.stamp.nanosec)
        out.point_num = n
        out.lidar_id = 0
        out.rsvd = [0, 0, 0]
        out.points = pts
        self.pub.publish(out)


def main():
    argv = sys.argv[1:]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    ap = argparse.ArgumentParser()
    ap.add_argument('-n', '--namespace', default='',
                    help='Rover namespace (e.g. rover_0). Empty = no namespace.')
    args = ap.parse_args(argv)

    rclpy.init()
    node = LivoxPublisher(args.namespace)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
