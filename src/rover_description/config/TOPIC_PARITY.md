# Topic parity: sim vs. real hardware

This is the reference sheet for what topics the sim publishes and subscribes to. It exists so Misha (Dev 2, SLAM) and Ivan (Dev 3, autonomy) know exactly what they can subscribe against today, and so nothing breaks when we swap the sim for real robots in Week 5.

If a topic name, type, or rate needs to change, edit this file in the same commit. This is the contract.

## How to read it

Direction column: "sim publishes" means something in Gazebo produces it and downstream nodes read it. "sim subscribes" means autonomy publishes it and the sim consumes it. QoS is RELIABLE unless noted; sensor topics prefer BEST_EFFORT (SensorDataQoS on the consumer side).

Rate is nominal (what the sim aims for); real hardware may vary +/- 20%.

## Per-rover topics (namespaced /rover_i/*)

### Motion and odometry

| Topic | Type | Direction | Rate | Real driver |
|-------|------|-----------|------|-------------|
| /rover_i/cmd_vel | geometry_msgs/Twist | sim subscribes | 20 Hz | motor controller node |
| /rover_i/odom | nav_msgs/Odometry | sim publishes | ~50 Hz | wheel encoders |
| /rover_i/joint_states | sensor_msgs/JointState | sim publishes | ~50 Hz | motor driver |

The odom frame is rover_i/odom, child is rover_i/base_link. That chain comes from the DiffDrive plugin.

### Perception

| Topic | Type | Direction | Rate | Real driver |
|-------|------|-----------|------|-------------|
| /rover_i/lidar/points | sensor_msgs/PointCloud2 | sim publishes | 5 Hz | livox_ros_driver2 (xfer_format 0) |
| /rover_i/livox/lidar | livox_msgs/CustomMsg | sim publishes | 5 Hz | livox_ros_driver2 (xfer_format 1) |
| /rover_i/lidar | sensor_msgs/LaserScan | sim publishes | 5 Hz | (sim legacy, plan to remove) |
| /rover_i/camera/image | sensor_msgs/Image | sim publishes | 15 Hz | LeTMC-520 |
| /rover_i/imu | sensor_msgs/Imu | sim publishes | 100 Hz | MPU-9250 or BMI088 |

The two lidar topics matter: use PointCloud2 for Cartographer, Nav2, and anything generic. Use CustomMsg for FAST-LIO2, Point-LIO, and LIO-SAM Livox variant. They carry the same underlying data.

### Sonars (six per rover: front, rear, left, right, front_left, front_right)

| Topic | Type | Direction | Rate | Real driver |
|-------|------|-----------|------|-------------|
| /rover_i/sonar/DIR | sensor_msgs/LaserScan | sim publishes | 5 Hz | (sim internal, raw scan) |
| /rover_i/sonar/DIR/range | sensor_msgs/Range | sim publishes | 5 Hz | HC-SR04 |

The DIR/range topic is what you should actually subscribe to. It comes from the sonar_to_range node that turns the raw scan into a proper cone.

### Robot state

| Topic | Type | Direction | Rate | Real driver |
|-------|------|-----------|------|-------------|
| /rover_i/robot_description | std_msgs/String | sim publishes (latched) | on start | robot_state_publisher |
| /rover_i/battery_state | sensor_msgs/BatteryState | sim publishes | 1 Hz | BMS |

## Fleet-wide topics (no namespace)

| Topic | Type | Direction | Rate | Notes |
|-------|------|-----------|------|-------|
| /clock | rosgraph_msgs/Clock | sim publishes | 1000 Hz | drives use_sim_time everywhere |
| /tf | tf2_msgs/TFMessage | shared | ~50 Hz | all rovers publish here, prefixed |
| /tf_static | tf2_msgs/TFMessage | latched | on start | sensor mount transforms |
| /emergency_stop | std_msgs/Bool | shared | 20 Hz | anyone can trigger, teleop must obey |
| /dock_0/status | std_msgs/String | sim publishes | 5 Hz | format: STATE\|rover_ns\|distance |
| /diagnostics | diagnostic_msgs/DiagnosticArray | sim publishes | 1 Hz | fleet health rollup |

## TF frame layout

Everything hangs off a shared `world` frame so multiple rovers can be viewed at once in RViz.

```
world
 rover_i/odom              (static, set at spawn)
   rover_i/base_link       (from DiffDrive)
     rover_i/lidar_link
     rover_i/camera_link
     rover_i/imu_link
     rover_i/sonar_DIR_link  (six of these)
     rover_i/wheel_XX_link    (four of these)
```

One gotcha: Ignition flattens fixed-joint frames, so a raw sensor's frame_id looks like `rover_i/base_link/lidar` instead of `rover_i/lidar_link`. The `sensor_frame_aliases.py` node publishes identity transforms so downstream code can use the clean names.

## Sensor noise

Currently no noise blocks in the URDF. We tried adding realistic Gaussian noise to lidar, RGB, IMU, and sonars but it dropped Gazebo's real-time factor enough to make physics stepping visibly jerky. Left off until Dev 2 / Dev 3 need it for SLAM tuning.

Reference values to use when adding it back:

| Sensor | Noise | Value | Source |
|--------|-------|-------|--------|
| Livox lidar range | Gaussian | 0.02 m stddev | Mid-360 datasheet: 2 cm at 25 m |
| RGB camera | Gaussian | 0.007 (normalized) | Consumer CMOS, roughly 1.8 grey levels on 8-bit |
| IMU gyro | Gaussian + bias walk | 0.001 rad/s, bias 0.0001 rad/s^2 per axis | MPU-9250 class |
| IMU accel | Gaussian + bias walk | 0.01 m/s^2, bias 0.001 m/s^3 per axis | Same |
| Sonar range | Gaussian | 0.003 m stddev | HC-SR04 spec, 3 mm |

Rule of thumb: sim noise should be a bit worse than the real datasheet. That way anything you build in sim survives the real robot being worse than the spec sheet claimed.

Practical note: adding all of these at once on a laptop-class CPU cost us about 30% of real-time factor at N=3 rovers. If you re-enable, do it one sensor at a time and watch the RTF gauge in the Gazebo GUI.

## use_sim_time is on for every node

If any node ran on wall clock while others ran on /clock, TF lookups would fail. Coverage check:

- parameter_bridge (per rover): set inline in launch
- robot_state_publisher (per rover): set inline in launch
- sonar_to_range, livox_publisher, sensor_frame_aliases: set inline
- battery_publisher, dock_monitor_0, estop_manager, fleet_teleop: set via swarm.yaml /** wildcard
- diagnostics_aggregator: set inline
- rviz2, global bridges, static TFs: set inline

Nothing runs on wall clock.

## Known gaps between sim and real

These are things sim gets wrong on purpose or by limitation. Recorded so nobody thinks the sim is perfect.

1. **Livox scan pattern.** Sim emits a uniform grid. Real Mid-360 sweeps a non-repeating rosette. The CustomMsg format is byte-identical; only the point distribution differs. FAST-LIO2 still runs, it just doesn't get the density-fill benefit. Deferred to Week 6 unless SLAM drifts.

2. **Bridge QoS is RELIABLE.** Our own converter nodes publish BEST_EFFORT correctly. The ros_gz_bridge underneath publishes RELIABLE by default. If you subscribe with SensorDataQoS, you get messages either way. To be strict we'd switch the bridge to config-file mode.

3. **Legacy /rover_i/lidar (LaserScan).** Still bridged for backward compatibility. New code should use /rover_i/lidar/points or /rover_i/livox/lidar.

4. **Perfect odometry.** Sim odom comes from the DiffDrive plugin, no drift, no slip. Real wheel odom drifts a lot. Ivan's IMU fusion is what closes that gap on real hardware; sim doesn't fake the drift.

## Changing this doc

- Adding a topic: add the row, name the real driver that will produce it.
- Removing a topic: deprecate for a week first, announce in the team channel.
- Changing a type or field name: this is a breaking change, get Misha and Ivan to sign off before merging.
- Changing a rate: update the row and mention it in the standup.
