#!/usr/bin/env python3
"""Dock monitor: watches rover odometry, publishes docking state per dock.

State machine per (rover, dock) pair:
  IDLE      — rover far from dock
  APPROACH  — within 1.0 m
  DOCKING   — within 0.30 m AND heading roughly reversed toward dock
  CHARGING  — within 0.10 m of contact plate (i.e. rover butted up against back wall)

Publishes /dock_<i>/status (std_msgs/String) at 5 Hz for whichever rover is closest.
Console log prints state transitions so you see charging kick in during teleop.

Kept intentionally simple — no plugin, no material swap. Real robotics puts
state on a topic; visual feedback is decorative and comes later.
"""
import argparse
import math
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def yaw_from_quat(q):
    """Extract yaw (Z rotation) from geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    """Smallest signed difference a-b in (-pi, pi]."""
    d = (a - b + math.pi) % (2.0 * math.pi) - math.pi
    return d


class DockMonitor(Node):
    def __init__(self, rover_namespaces):
        # Node name matches the YAML block header so params load cleanly.
        super().__init__('dock_monitor_0',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        # Params come from config/swarm.yaml (dock_monitor_0 block).
        self.dock_id = int(self._param('dock_id', 0))
        self.dock_x = float(self._param('dock_x', -2.0))
        self.dock_y = float(self._param('dock_y', 0.0))
        self.dock_yaw = float(self._param('dock_yaw', 0.0))
        contact_offset = float(self._param('contact_offset', 0.15))
        self.charging_radius = float(self._param('charging_radius', 0.10))
        self.charging_heading_tol = float(self._param('charging_heading_tol', 0.35))
        self.docking_radius = float(self._param('docking_radius', 0.30))
        self.docking_heading_tol = float(self._param('docking_heading_tol', 0.60))
        self.approach_radius = float(self._param('approach_radius', 1.0))

        # Back plate is contact_offset metres behind dock origin along -X_dock.
        cx = self.dock_x - contact_offset * math.cos(self.dock_yaw)
        cy = self.dock_y - contact_offset * math.sin(self.dock_yaw)
        self.contact_x, self.contact_y = cx, cy

        self.rovers = {}  # ns -> (x, y, yaw)
        for ns in rover_namespaces:
            topic = f'/{ns}/odom'
            self.create_subscription(Odometry, topic, self._make_cb(ns), 10)
            self.rovers[ns] = None
            self.get_logger().info(f'watching {topic}')

        self.pub = self.create_publisher(String, f'/dock_{self.dock_id}/status', 10)
        self.last_state = {ns: 'IDLE' for ns in rover_namespaces}
        self.timer = self.create_timer(0.2, self.tick)  # 5 Hz

    def _param(self, name, default):
        # Guard against declared-but-empty parameters (value None) that would
        # otherwise crash float()/int() in the caller.
        try:
            val = self.get_parameter(name).value
        except Exception:
            return default
        return default if val is None else val

    def _make_cb(self, ns):
        def cb(msg: Odometry):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.rovers[ns] = (p.x, p.y, yaw_from_quat(q))
        return cb

    def classify(self, x, y, yaw):
        """Return (state, distance_to_contact, heading_error)."""
        dx = x - self.contact_x
        dy = y - self.contact_y
        dist = math.hypot(dx, dy)
        # Heading should match dock heading for reverse-in (rover backs into throat).
        heading_err = abs(angle_diff(yaw, self.dock_yaw))

        if dist < self.charging_radius and heading_err < self.charging_heading_tol:
            return 'CHARGING', dist, heading_err
        if dist < self.docking_radius and heading_err < self.docking_heading_tol:
            return 'DOCKING', dist, heading_err
        if dist < self.approach_radius:
            return 'APPROACH', dist, heading_err
        return 'IDLE', dist, heading_err

    def tick(self):
        best_ns, best_state, best_dist = None, 'IDLE', float('inf')
        for ns, pose in self.rovers.items():
            if pose is None:
                continue
            state, dist, herr = self.classify(*pose)
            # Print per-rover transitions
            if state != self.last_state[ns]:
                self.get_logger().info(
                    f'{ns}: {self.last_state[ns]} -> {state}  '
                    f'(dist={dist:.2f} m, heading_err={math.degrees(herr):.0f}°)'
                )
                self.last_state[ns] = state
            # Choose the "most engaged" rover for the dock's public status.
            priority = {'IDLE': 0, 'APPROACH': 1, 'DOCKING': 2, 'CHARGING': 3}
            if priority[state] > priority[best_state] or \
               (priority[state] == priority[best_state] and dist < best_dist):
                best_ns, best_state, best_dist = ns, state, dist

        msg = String()
        msg.data = f'{best_state}|{best_ns or "-"}|{best_dist:.3f}'
        self.pub.publish(msg)


def main():
    argv = sys.argv[1:]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    ap = argparse.ArgumentParser()
    ap.add_argument('--rovers', nargs='+', default=['rover_0'],
                    help='Rover namespaces to watch (space-separated)')
    args = ap.parse_args(argv)

    rclpy.init()
    node = DockMonitor(args.rovers)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
