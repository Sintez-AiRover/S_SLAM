#!/usr/bin/env python3
"""Battery state publisher — simulates per-rover battery for the swarm.

Publishes /<ns>/battery_state (sensor_msgs/BatteryState) at 1 Hz.

State model:
  - Draws drive_current when |twist.linear.x| + |twist.angular.z| > 0
  - Draws idle_current otherwise
  - Charges at +charge_current when /dock_0/status shows this rover is CHARGING

All rates and capacity come from config/swarm.yaml (battery_publisher block).
Real BMS driver will replace this node in Week 5 hardware bringup, but the
topic contract stays identical — Ivan's autonomy subscribes to the same topic.
"""
import argparse
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class BatteryPublisher(Node):
    def __init__(self, namespace):
        super().__init__('battery_publisher',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.ns = namespace

        # Read tuning from swarm.yaml (loaded by launch as node params).
        self.capacity_wh = float(self._param('capacity_wh', 100.0))
        self.nominal_voltage = float(self._param('nominal_voltage', 24.0))
        self.drive_current = float(self._param('drive_current', 3.0))
        self.idle_current = float(self._param('idle_current', 0.5))
        self.charge_current = float(self._param('charge_current', 10.0))
        self.publish_rate = float(self._param('publish_rate', 1.0))
        percentage = float(self._param('initial_percentage', 0.85))

        # Charge in Ah = capacity_wh / voltage. State-of-charge is fraction remaining.
        self.capacity_ah = self.capacity_wh / self.nominal_voltage
        self.charge_ah = self.capacity_ah * percentage

        self.last_twist = Twist()
        self.charging = False

        best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST, depth=5)
        cmd_topic = f'/{namespace}/cmd_vel' if namespace else '/cmd_vel'
        bat_topic = f'/{namespace}/battery_state' if namespace else '/battery_state'

        self.create_subscription(Twist, cmd_topic, self._on_twist, 10)
        # Dock status is a fleet-wide topic; parse to check if THIS rover is charging.
        self.create_subscription(String, '/dock_0/status', self._on_dock, 10)
        self.pub = self.create_publisher(BatteryState, bat_topic, best_effort)

        period = 1.0 / max(0.1, self.publish_rate)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'battery: {bat_topic}  capacity={self.capacity_wh:.1f} Wh  start={percentage*100:.0f}%'
        )

    def _param(self, name, default):
        # get_parameter can return a declared-but-empty parameter whose .value is
        # None (happens when the YAML block was declared for the node but the
        # override didn't match this node's fully-qualified name). Fall back to
        # default in that case, don't let None reach float()/int() constructors.
        try:
            val = self.get_parameter(name).value
        except Exception:
            return default
        return default if val is None else val

    def _on_twist(self, msg: Twist):
        self.last_twist = msg

    def _on_dock(self, msg: String):
        # Format: STATE|rover_ns|distance
        parts = msg.data.split('|')
        if len(parts) >= 2:
            state, ns = parts[0], parts[1]
            self.charging = (state == 'CHARGING' and ns == self.ns)

    def _tick(self):
        # Integrate one publish-period of current draw.
        dt_h = (1.0 / max(0.1, self.publish_rate)) / 3600.0  # seconds → hours
        speed = abs(self.last_twist.linear.x) + abs(self.last_twist.angular.z)
        if self.charging:
            current = -self.charge_current   # negative = charging (BatteryState convention)
        elif speed > 0.01:
            current = self.drive_current
        else:
            current = self.idle_current

        self.charge_ah -= current * dt_h
        self.charge_ah = max(0.0, min(self.capacity_ah, self.charge_ah))

        bat = BatteryState()
        bat.header.stamp = self.get_clock().now().to_msg()
        bat.header.frame_id = f'{self.ns}/base_link' if self.ns else 'base_link'
        bat.voltage = self.nominal_voltage
        bat.current = -current  # BatteryState: + = discharging out, - = charging in
        bat.charge = float(self.charge_ah)
        bat.capacity = float(self.capacity_ah)
        bat.design_capacity = float(self.capacity_ah)
        bat.percentage = float(self.charge_ah / self.capacity_ah)
        bat.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING if self.charging
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        bat.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        bat.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        bat.present = True
        self.pub.publish(bat)


def main():
    argv = sys.argv[1:]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    ap = argparse.ArgumentParser()
    ap.add_argument('-n', '--namespace', default='')
    args = ap.parse_args(argv)

    rclpy.init()
    node = BatteryPublisher(args.namespace)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
