#!/usr/bin/env python3
"""Broadcast the 10 identity static TFs that alias URDF sensor links → Ignition's
flattened sensor frame_ids, all from ONE process.

Why one node instead of 10 static_transform_publisher nodes: at N=10 rovers that's
100 rclpy processes just for TF aliasing — each ~40–60 MB. This node uses a single
`StaticTransformBroadcaster` to publish all aliases for one rover in one shot.

Ignition merges fixed-joint child links into the parent during URDF→SDF conversion,
so sensors publish frame_id `<name>/base_link/<sensor_name>` instead of e.g.
`lidar_link`. We add identity static TFs from each URDF link (as prefixed by
robot_state_publisher's `frame_prefix`) → its Ignition-scoped name.
"""
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

# (URDF link name, Ignition-scoped sensor frame suffix)
ALIASES = [
    ('lidar_link',             'lidar'),
    ('camera_link',             'camera'),
    ('camera_link',             'thermal'),
    ('base_link',               'imu'),
    ('sonar_front_link',        'sonar_front'),
    ('sonar_front_right_link',  'sonar_front_right'),
    ('sonar_front_left_link',   'sonar_front_left'),
    ('sonar_rear_link',         'sonar_rear'),
    ('sonar_rear_right_link',   'sonar_rear_right'),
    ('sonar_rear_left_link',    'sonar_rear_left'),
]


class SensorFrameAliases(Node):
    def __init__(self):
        super().__init__('sensor_frame_aliases')
        self.declare_parameter('namespace', '')
        ns = self.get_parameter('namespace').get_parameter_value().string_value
        frame_prefix = f'{ns}/' if ns else ''
        # Ignition scopes sensor frames under the spawned model name.
        ign_prefix = f'{ns}/' if ns else ''

        br = StaticTransformBroadcaster(self)
        stamps = []
        now = self.get_clock().now().to_msg()
        for urdf_link, ign_suffix in ALIASES:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = f'{frame_prefix}{urdf_link}'
            t.child_frame_id = f'{ign_prefix}base_link/{ign_suffix}'
            t.transform.rotation.w = 1.0  # identity
            stamps.append(t)
        br.sendTransform(stamps)
        self.get_logger().info(
            f'[{ns or "global"}] published {len(stamps)} sensor-frame aliases'
        )


def main():
    rclpy.init()
    node = SensorFrameAliases()
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
