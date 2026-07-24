#!/usr/bin/env python3
"""Multi-rover teleop — single terminal, digit keys switch active rover.

Usage:
    rover_teleop.py --rovers rover_0 rover_1 rover_2 [--spawn-spacing 0.8]

Legacy single-rover form still works:
    rover_teleop.py -n rover_0 -y 0.0        # exactly one rover

Controls:
  arrows        drive / turn ACTIVE rover
  SPACE         stop ACTIVE rover
  1..9          switch active rover (index into --rovers list, 1-based)
  b             broadcast toggle — same twist to ALL rovers (formation drive)
  q / a         active rover linear speed +20% / -20%
  w / s         active rover turn rate   +20% / -20%
  R             reset active rover to its spawn pose
  ?             reprint help
  Ctrl+C        quit

Design pattern: one process, one publisher per rover, one active-rover index.
Mirrors Clearpath's multi_teleop_keyboard and Nav2 fleet-example teleops —
the professional pattern for R&D multi-rover control (one operator, one
window, switch focus with digits). Real fleet ops move to Foxglove Studio
or a gamepad; this is the R&D bridge.

Motion tuning is set at the top of this file. IMPORTANT: LIN_ACCEL and
ANG_ACCEL MUST equal the URDF DiffDrive plugin's max_linear_acceleration
and max_angular_acceleration — otherwise you get constant jerk from
double-filtering. Warning comments in both files.
"""
import argparse
import sys
import select
import termios
import tty
import time
import subprocess
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

SPEED_INIT = 0.56   # initial linear speed in m/s — URDF caps at max_linear_velocity=2.0
TURN_INIT  = 0.56   # initial turn rate in rad/s — URDF caps at max_angular_velocity=2.0
SPEED_MIN, SPEED_MAX = 0.56, 1.0
TURN_MIN,  TURN_MAX  = 0.56, 1.0
SPEED_STEP = 1.2
TIMEOUT = 0.6
# CRITICAL: these MUST match the URDF DiffDrive limits or you get constant jerk
# from double-filtering. Currently URDF has max_linear_acceleration=2.0,
# max_angular_acceleration=4.0. Change both together if you re-tune.
LIN_ACCEL = 1.0
ANG_ACCEL = 2.0

PUBLISH_PERIOD = 0.05  # 20 Hz

ARROW_UP, ARROW_DOWN, ARROW_RIGHT, ARROW_LEFT = '\x1b[A', '\x1b[B', '\x1b[C', '\x1b[D'


def get_key(poll=PUBLISH_PERIOD):
    r, _, _ = select.select([sys.stdin], [], [], poll)
    if not r:
        return ''
    key = sys.stdin.read(1)
    if key == '\x1b':
        # Arrow keys are ESC [ X. Follow-up bytes normally arrive within a few ms,
        # but under load gnome-terminal can lag. 0.25 s is comfortably above any
        # realistic terminal jitter without making bare-ESC feel unresponsive.
        # If ONLY ESC is received, we still drain any trailing bytes so a leftover
        # 'B' or 'D' from a partial arrow doesn't hit the next loop as a bare key.
        for _ in range(2):
            r2, _, _ = select.select([sys.stdin], [], [], 0.25)
            if not r2:
                break
            key += sys.stdin.read(1)
    return key


def reset_rover(name, x=0.0, y=0.0):
    subprocess.run([
        'ign', 'service', '-s', '/world/rover_world/set_pose',
        '--reqtype', 'ignition.msgs.Pose',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '500',
        '--req', f'name: "{name}", position: {{x: {x}, y: {y}, z: 0.2}}, orientation: {{w: 1}}',
    ], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class RoverState:
    """Per-rover: publisher + ramp state + reset pose."""
    def __init__(self, node, namespace, spawn_x, spawn_y):
        self.namespace = namespace
        self.spawn_x = spawn_x
        self.spawn_y = spawn_y
        topic = f'/{namespace}/cmd_vel' if namespace else '/cmd_vel'
        self.pub = node.create_publisher(Twist, topic, 10)
        self.target_lin = 0.0
        self.target_ang = 0.0
        self.cur_lin = 0.0
        self.cur_ang = 0.0
        self.last_press = 0.0
        # Per-rover live-adjustable speed profile (q/a/w/s modify the ACTIVE rover only).
        self.speed = SPEED_INIT
        self.turn = TURN_INIT

    def tick(self, now):
        """Enforce timeout, ramp toward target, publish."""
        if self.last_press and (now - self.last_press) > TIMEOUT:
            self.target_lin, self.target_ang = 0.0, 0.0
            self.last_press = 0.0
        dv = LIN_ACCEL * PUBLISH_PERIOD
        dw = ANG_ACCEL * PUBLISH_PERIOD
        self.cur_lin += max(-dv, min(dv, self.target_lin - self.cur_lin))
        self.cur_ang += max(-dw, min(dw, self.target_ang - self.cur_ang))
        t = Twist()
        t.linear.x = self.cur_lin
        t.angular.z = self.cur_ang
        self.pub.publish(t)


def print_help(rovers, active_idx, broadcast):
    slots = '  '.join(
        f'[{i+1}]{r.namespace}{"*" if i == active_idx else ""}'
        for i, r in enumerate(rovers)
    )
    active = rovers[active_idx]
    mode = 'BROADCAST (all)' if broadcast else f'active: {active.namespace}'
    sys.stdout.write(
        f'\r\n=== Rover teleop  →  {mode} ===\r\n'
        f'  slots: {slots}   (* = active)\r\n'
        f'  active speed={active.speed:.2f} m/s  turn={active.turn:.2f} rad/s\r\n'
        '  arrows drive/turn   SPACE stop   1..9 select rover   b broadcast toggle\r\n'
        '  q/a speed ±20%   w/s turn ±20%   R reset active rover   ? help   Ctrl+C quit\r\n'
    )
    sys.stdout.flush()


def main():
    argv = sys.argv[1:]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    ap = argparse.ArgumentParser()
    ap.add_argument('--rovers', nargs='+', default=None,
                    help='Multi-rover mode: space-separated namespaces (e.g. rover_0 rover_1 rover_2)')
    ap.add_argument('--spawn-poses', nargs='+', default=None,
                    help='Per-rover spawn poses as "x,y" pairs, aligned with --rovers. '
                         'Comes from swarm.yaml via the launch file; used by the R reset key. '
                         'Overrides --spawn-spacing.')
    ap.add_argument('--spawn-spacing', type=float, default=0.8,
                    help='Fallback Y spacing between rovers when --spawn-poses is absent. Default 0.8.')
    # Legacy single-rover flags (kept so old launch invocations still work).
    ap.add_argument('-n', '--namespace', default='')
    ap.add_argument('-x', '--spawn-x', type=float, default=0.0)
    ap.add_argument('-y', '--spawn-y', type=float, default=0.0)
    args = ap.parse_args(argv)

    rclpy.init()
    node = rclpy.create_node('rover_teleop')

    # E-stop subscription: while True, teleop skips publishing so it doesn't fight
    # the estop_manager's zero-twist. This is the "downstream nodes MUST subscribe"
    # side of the fleet-wide E-stop contract.
    estop_state = {'stopped': False}
    def _on_estop(msg):
        estop_state['stopped'] = msg.data
    node.create_subscription(Bool, '/emergency_stop', _on_estop, 10)

    # Build the rover roster. Multi-rover form wins if provided; else fall back to legacy single.
    if args.rovers:
        if args.spawn_poses and len(args.spawn_poses) == len(args.rovers):
            spawn_xy = [tuple(float(v) for v in p.split(',')[:2]) for p in args.spawn_poses]
        else:
            if args.spawn_poses:
                sys.stderr.write('rover_teleop: --spawn-poses count != --rovers count; '
                                 'falling back to --spawn-spacing line layout\n')
            spawn_xy = [(args.spawn_x, i * args.spawn_spacing) for i in range(len(args.rovers))]
        rovers = [
            RoverState(node, ns, *spawn_xy[i])
            for i, ns in enumerate(args.rovers)
        ]
    else:
        rovers = [RoverState(node, args.namespace, args.spawn_x, args.spawn_y)]

    active_idx = 0
    broadcast = False

    print_help(rovers, active_idx, broadcast)

    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    try:
        while True:
            key = get_key(poll=PUBLISH_PERIOD)
            now = time.time()
            active = rovers[active_idx]

            # Digit → switch active rover
            if key in tuple('123456789'):
                idx = int(key) - 1
                if 0 <= idx < len(rovers):
                    active_idx = idx
                    print_help(rovers, active_idx, broadcast)
            elif key == 'b':
                # LOWERCASE only. Uppercase 'B' is the tail of arrow-DOWN's ESC[B
                # sequence — accepting it would false-trigger broadcast when the
                # user just drives backward.
                broadcast = not broadcast
                print_help(rovers, active_idx, broadcast)
            elif key == '?':
                print_help(rovers, active_idx, broadcast)
            elif key == ARROW_UP:
                targets = rovers if broadcast else [active]
                for r in targets:
                    r.target_lin, r.target_ang = r.speed, 0.0
                    r.last_press = now
            elif key == ARROW_DOWN:
                targets = rovers if broadcast else [active]
                for r in targets:
                    r.target_lin, r.target_ang = -r.speed, 0.0
                    r.last_press = now
            elif key == ARROW_LEFT:
                targets = rovers if broadcast else [active]
                for r in targets:
                    r.target_lin, r.target_ang = 0.0, r.turn
                    r.last_press = now
            elif key == ARROW_RIGHT:
                targets = rovers if broadcast else [active]
                for r in targets:
                    r.target_lin, r.target_ang = 0.0, -r.turn
                    r.last_press = now
            elif key == ' ':
                targets = rovers if broadcast else [active]
                for r in targets:
                    r.target_lin, r.target_ang = 0.0, 0.0
                    r.last_press = 0.0
            elif key in ('q', 'Q'):
                active.speed = min(SPEED_MAX, active.speed * SPEED_STEP)
                sys.stdout.write(f'\r  {active.namespace} speed = {active.speed:.2f} m/s\r\n'); sys.stdout.flush()
            elif key in ('a', 'A'):
                active.speed = max(SPEED_MIN, active.speed / SPEED_STEP)
                sys.stdout.write(f'\r  {active.namespace} speed = {active.speed:.2f} m/s\r\n'); sys.stdout.flush()
            elif key in ('w', 'W'):
                active.turn = min(TURN_MAX, active.turn * SPEED_STEP)
                sys.stdout.write(f'\r  {active.namespace} turn  = {active.turn:.2f} rad/s\r\n'); sys.stdout.flush()
            elif key in ('s', 'S'):
                active.turn = max(TURN_MIN, active.turn / SPEED_STEP)
                sys.stdout.write(f'\r  {active.namespace} turn  = {active.turn:.2f} rad/s\r\n'); sys.stdout.flush()
            elif key == 'r':
                # LOWERCASE only. Uppercase 'R' avoids collision with any stray
                # follow-byte from a garbled escape sequence.
                name = active.namespace if active.namespace else 'rover'
                reset_rover(name, active.spawn_x, active.spawn_y)
                sys.stdout.write(f'\r  {name} reset to ({active.spawn_x:.1f}, {active.spawn_y:.1f})\r\n'); sys.stdout.flush()
            elif key == '\x03':  # Ctrl+C
                break

            # Pump ROS callbacks (e-stop subscription).
            rclpy.spin_once(node, timeout_sec=0.0)

            # Every rover ticks. If e-stop is engaged, publish zero-twist instead
            # of the ramped target — the estop_manager already floods zeros, but
            # this makes the intent explicit at the teleop layer too.
            if estop_state['stopped']:
                zero = Twist()
                for r in rovers:
                    r.target_lin = 0.0
                    r.target_ang = 0.0
                    r.cur_lin = 0.0
                    r.cur_ang = 0.0
                    r.pub.publish(zero)
            else:
                for r in rovers:
                    r.tick(now)

    finally:
        # Publish a final zero-twist to every rover so nothing keeps rolling on exit.
        for r in rovers:
            r.pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
