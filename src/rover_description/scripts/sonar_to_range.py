#!/usr/bin/env python3
"""Convert 6 sonar LaserScan topics into sensor_msgs/Range messages.

Why: Ignition's gpu_lidar publishes LaserScan (an array of N range samples across an
angular arc). RViz renders LaserScan as scattered points — visually useless for a
proximity sensor. sensor_msgs/Range, in contrast, RViz renders as a translucent cone
with length = measured distance. That's the conventional and correct representation
for an ultrasonic-style proximity sensor in a sensor's *local* frame.

Multi-robot: pass a `namespace` param (e.g. `rover_0`). All input topics become
`/<ns>/sonar/*`, output topics `/<ns>/sonar/*/range`, and frame_ids are prefixed
`<ns>/sonar_*_link` to match robot_state_publisher's `frame_prefix`.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, Range

# Each entry: (bare topic name, bare frame link name)
SONARS = [
    ('front',       'sonar_front_link'),
    ('front_right', 'sonar_front_right_link'),
    ('front_left',  'sonar_front_left_link'),
    ('rear',        'sonar_rear_link'),
    ('rear_right',  'sonar_rear_right_link'),
    ('rear_left',   'sonar_rear_left_link'),
]

FOV = 0.4363   # 25° cone — matches URDF gpu_lidar horizontal span
MIN_RANGE = 0.02
MAX_RANGE = 2.0


class SonarToRange(Node):
    def __init__(self):
        super().__init__('sonar_to_range')
        self.declare_parameter('namespace', '')
        ns = self.get_parameter('namespace').get_parameter_value().string_value
        prefix = f'/{ns}' if ns else ''
        frame_prefix = f'{ns}/' if ns else ''

        # Subscribe BEST_EFFORT (matches Ignition bridge input side); publish
        # RELIABLE so RViz Range displays (default RELIABLE) actually receive
        # messages. Sonar cones are ~5 Hz — no perf reason for BEST_EFFORT out.
        # High-frequency topics (Livox lidar, camera) stay BEST_EFFORT out.
        sub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pubs = {}
        for bare_topic, bare_frame in SONARS:
            in_topic  = f'{prefix}/sonar/{bare_topic}'
            out_topic = f'{prefix}/sonar/{bare_topic}/range'
            frame     = f'{frame_prefix}{bare_frame}'
            self._pubs[in_topic] = (
                self.create_publisher(Range, out_topic, pub_qos),
                frame,
            )
            self.create_subscription(
                LaserScan, in_topic,
                lambda msg, t=in_topic: self._cb(msg, t),
                sub_qos,
            )
        self.get_logger().info(
            f'[{ns or "global"}] converting {len(SONARS)} sonar LaserScans → Range'
        )

    def _cb(self, scan: LaserScan, in_topic: str):
        pub, frame = self._pubs[in_topic]
        # Filter out inf / NaN / out-of-band returns.
        valid = [r for r in scan.ranges
                 if math.isfinite(r) and MIN_RANGE <= r <= MAX_RANGE]
        # If nothing in range, report max_range — RViz then draws the full cone,
        # which is the conventional "no obstacle" rendering.
        r = min(valid) if valid else MAX_RANGE

        msg = Range()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = frame
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = FOV
        msg.min_range = MIN_RANGE
        msg.max_range = MAX_RANGE
        msg.range = float(r)
        pub.publish(msg)


def main():
    rclpy.init()
    node = SonarToRange()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
