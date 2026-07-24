#!/bin/bash
# Clean start for the S3E Swarm-SLAM experiment.
# Kills any leftover nodes from previous runs first (leftovers publish on the
# same topics and corrupt the new run's map), then launches everything.
# Usage: ./start_s3e.sh [extra launch args, e.g. rate:=0.25]

pkill -9 -f 's3e_stereo_and_lidar' 2>/dev/null
pkill -9 -f 'icp_odometry' 2>/dev/null
pkill -9 -f 'cslam_map_manager|map_manager' 2>/dev/null
pkill -9 -f 'pose_graph_manager' 2>/dev/null
pkill -9 -f 'loop_closure_detection' 2>/dev/null
pkill -9 -f 's3e_camera_pipeline' 2>/dev/null
pkill -9 -f 's3e_map_accumulator' 2>/dev/null
pkill -9 -f 'static_transform_publisher' 2>/dev/null
# 'ros2 bag play' does not contain the string "rosbag2", match it explicitly
pkill -9 -f 'ros2 bag play' 2>/dev/null
pkill -9 -f 'rosbag2' 2>/dev/null
pkill -9 -f 'rviz2' 2>/dev/null
pkill -9 -f 'visualization_node' 2>/dev/null
sleep 2

LEFT=$(pgrep -cf 'icp_odometry|map_manager|camera_pipeline|rosbag2|rviz2|accumulator' || true)
if [ "${LEFT:-0}" -gt 0 ]; then
    echo "WARNING: $LEFT leftover processes still alive, waiting 3 s and killing again..."
    sleep 3
    pkill -9 -f 'icp_odometry|map_manager|camera_pipeline|rosbag2|rviz2|accumulator' 2>/dev/null
    sleep 1
fi
echo "Clean. Launching Swarm-SLAM on S3E..."

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# Isolate from other ROS 2 systems on the network: another machine on the
# LAN (was 10.1.16.127) ran a Gazebo sim whose ros_gz_bridge publishes
# /clock starting at 0 s on the default domain 0; DDS auto-discovery pulled
# it in, and it interleaved with the bag's 2022-epoch clock making RViz/tf
# reset on every tick (blinking displays, odometry frame skips). A private
# domain ignores all domain-0 traffic, local or remote. Any terminal used
# to inspect this experiment (ros2 topic echo, etc.) needs the same export.
# NOTE: do NOT add ROS_LOCALHOST_ONLY=1 — with CycloneDDS it caps discovery
# at ~10 participants per host and this launch spawns ~25 processes; the
# excess nodes die with "rmw_create_node: failed to create domain".
export ROS_DOMAIN_ID=42
exec ros2 launch cslam_experiments s3e_stereo_and_lidar.launch.py max_nb_robots:=3 rate:=0.5 "$@"
