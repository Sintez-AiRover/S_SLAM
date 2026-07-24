#!/usr/bin/env python3
"""Fleet diagnostics aggregator.

Publishes /diagnostics (diagnostic_msgs/DiagnosticArray) at 1 Hz. Watches per-rover:
  - Battery: OK (>25%) / WARN (10-25%) / ERROR (<10%)
  - Odometry age: OK (<1s) / WARN (1-3s) / ERROR (>3s)
  - Livox lidar age: OK (<0.5s) / WARN (0.5-2s) / STALE (>2s)
  - Dock status: informational

Fleet-monitoring convention: real robot fleets publish /diagnostics from every
node; rqt_diagnostics_viewer and Foxglove read this topic. This is the minimal
aggregator that turns raw topic activity into KEY/VALUE/LEVEL rows.

Real nodes should use diagnostic_updater::Updater to self-report; this
aggregator polls topic freshness as a safety net for nodes we don't own
(the Ignition bridge etc.).
"""
import argparse
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from livox_msgs.msg import CustomMsg


# DiagnosticStatus.level is a `byte` field — rosidl Python bindings expect
# a single-byte bytes object, not an int. Constructing with int raises
# AssertionError('The level field must be of type bytes or ByteString').
OK    = bytes([0])
WARN  = bytes([1])
ERROR = bytes([2])
STALE = bytes([3])


class DiagAggregator(Node):
    def __init__(self, rovers):
        super().__init__('diagnostics_aggregator')
        self.rovers = rovers
        # Per-rover last-seen timestamps and last-values.
        self.state = {ns: {
            'battery_pct': None,
            'odom_stamp': 0.0,
            'lidar_stamp': 0.0,
        } for ns in rovers}
        self.dock_status = 'IDLE|-|inf'

        best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST, depth=5)

        for ns in rovers:
            self.create_subscription(BatteryState, f'/{ns}/battery_state',
                                     self._mk_bat(ns), best_effort)
            self.create_subscription(Odometry, f'/{ns}/odom',
                                     self._mk_odom(ns), 10)
            self.create_subscription(CustomMsg, f'/{ns}/livox/lidar',
                                     self._mk_lidar(ns), best_effort)
        self.create_subscription(String, '/dock_0/status', self._on_dock, 10)

        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0, self._publish)  # 1 Hz — convention
        self.get_logger().info(f'diagnostics_aggregator watching {rovers}')

    def _mk_bat(self, ns):
        def cb(m: BatteryState):
            self.state[ns]['battery_pct'] = m.percentage
        return cb

    def _mk_odom(self, ns):
        def cb(m: Odometry):
            self.state[ns]['odom_stamp'] = time.time()
        return cb

    def _mk_lidar(self, ns):
        def cb(m: CustomMsg):
            self.state[ns]['lidar_stamp'] = time.time()
        return cb

    def _on_dock(self, m: String):
        self.dock_status = m.data

    def _publish(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        now_t = time.time()

        for ns in self.rovers:
            s = self.state[ns]

            # Battery
            level = STALE
            msg = 'no battery message received'
            values = []
            if s['battery_pct'] is not None:
                pct = s['battery_pct']
                values = [KeyValue(key='percentage', value=f'{pct*100:.1f}%')]
                if pct < 0.10:
                    level, msg = ERROR, f'critical: {pct*100:.1f}%'
                elif pct < 0.25:
                    level, msg = WARN, f'low: {pct*100:.1f}%'
                else:
                    level, msg = OK, f'nominal: {pct*100:.1f}%'
            arr.status.append(DiagnosticStatus(
                level=level, name=f'{ns}: battery', message=msg,
                hardware_id=ns, values=values))

            # Odometry freshness
            age = now_t - s['odom_stamp'] if s['odom_stamp'] else float('inf')
            if age > 3.0:
                level, msg = STALE, f'no odom for {age:.1f}s'
            elif age > 1.0:
                level, msg = WARN, f'odom stale: {age:.1f}s'
            else:
                level, msg = OK, f'odom age {age:.2f}s'
            arr.status.append(DiagnosticStatus(
                level=level, name=f'{ns}: odometry', message=msg,
                hardware_id=ns, values=[KeyValue(key='age_s', value=f'{age:.3f}')]))

            # Livox lidar freshness
            age = now_t - s['lidar_stamp'] if s['lidar_stamp'] else float('inf')
            if age > 2.0:
                level, msg = STALE, f'no livox for {age:.1f}s'
            elif age > 0.5:
                level, msg = WARN, f'livox stale: {age:.2f}s'
            else:
                level, msg = OK, f'livox age {age:.2f}s'
            arr.status.append(DiagnosticStatus(
                level=level, name=f'{ns}: livox_lidar', message=msg,
                hardware_id=ns, values=[KeyValue(key='age_s', value=f'{age:.3f}')]))

        # Dock (single, fleet-wide)
        arr.status.append(DiagnosticStatus(
            level=OK, name='dock_0', message=self.dock_status,
            hardware_id='dock_0',
            values=[KeyValue(key='status', value=self.dock_status)]))

        self.pub.publish(arr)


def main():
    argv = sys.argv[1:]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    ap = argparse.ArgumentParser()
    ap.add_argument('--rovers', nargs='+', required=True)
    args = ap.parse_args(argv)

    rclpy.init()
    node = DiagAggregator(args.rovers)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
