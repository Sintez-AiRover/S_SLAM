#!/usr/bin/env python3
"""E-stop manager — fleet-wide emergency stop.

Publishes /emergency_stop (std_msgs/Bool). When True:
  - Immediately publishes zero-twist to every rover's /<ns>/cmd_vel
  - Continues publishing zero-twist at 20 Hz until released
  - Downstream nodes (teleop, autonomy) MUST subscribe to /emergency_stop and
    stop sending their own cmd_vel while it's True. This node's zero-twist
    is a last-line safety layer, not the primary mechanism.

Trigger sources (any of):
  - Keyboard: press E in the running terminal (raw mode)
  - Programmatic: publish True to /emergency_stop from any other node/tool
  - Service call: /trigger_estop (std_srvs/Trigger)  [future]

Release: press R (reset) in the terminal, or publish False.

Non-functional gate 'Robustness: survive single-robot failure' in the roadmap
demands this exists — real fleet reviews always ask 'where's the E-stop.'
"""
import argparse
import select
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class EstopManager(Node):
    def __init__(self, rovers, stop_topic):
        super().__init__('estop_manager')
        self.rovers = rovers
        self.stopped = False

        self.status_pub = self.create_publisher(Bool, stop_topic, 10)
        self.rover_pubs = [
            self.create_publisher(Twist, f'/{ns}/cmd_vel', 10) for ns in rovers
        ]

        # Any external node can also toggle by publishing to /emergency_stop.
        self.create_subscription(Bool, stop_topic, self._on_ext, 10)

        # Broadcast at 20 Hz while stopped so late-joining consumers see it.
        self.create_timer(0.05, self._tick)
        # Keyboard poller at 10 Hz.
        self.create_timer(0.1, self._poll_keys)

        # Enter raw stdin mode so single keypresses trigger the estop.
        self._settings = termios.tcgetattr(sys.stdin) if sys.stdin.isatty() else None
        if self._settings:
            tty.setcbreak(sys.stdin.fileno())

        self._banner()

    def _banner(self):
        self.get_logger().info(
            f'E-stop active on {self.status_pub.topic}  |  rovers={self.rovers}  '
            f'|  press E to stop, R to release, Ctrl+C to quit'
        )

    def _on_ext(self, msg: Bool):
        # Someone else published a state — track it.
        if msg.data != self.stopped:
            self.stopped = msg.data
            self.get_logger().warn(f'E-STOP {"ENGAGED" if msg.data else "released"} (external)')

    def _poll_keys(self):
        if not self._settings:
            return
        r, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not r:
            return
        ch = sys.stdin.read(1)
        if ch in ('e', 'E'):
            if not self.stopped:
                self.stopped = True
                self.get_logger().warn('E-STOP ENGAGED (keyboard)')
        elif ch in ('r', 'R'):
            if self.stopped:
                self.stopped = False
                self.get_logger().warn('E-STOP released (keyboard)')

    def _tick(self):
        # Always publish current stop state (so late subscribers pick it up).
        msg = Bool(); msg.data = self.stopped
        self.status_pub.publish(msg)
        if self.stopped:
            zero = Twist()
            for p in self.rover_pubs:
                p.publish(zero)

    def destroy_node(self):
        if self._settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)
        super().destroy_node()


def main():
    argv = sys.argv[1:]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    ap = argparse.ArgumentParser()
    ap.add_argument('--rovers', nargs='+', required=True)
    ap.add_argument('--stop-topic', default='/emergency_stop')
    args = ap.parse_args(argv)

    rclpy.init()
    node = EstopManager(args.rovers, args.stop_topic)
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
