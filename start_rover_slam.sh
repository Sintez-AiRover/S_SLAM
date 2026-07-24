#!/bin/bash
# Swarm-SLAM (cslam) on the AiRover simulation -- terminal 2 of 2.
# Start AFTER ./start_rover_sim.sh is up (needs /clock and the rover topics).
#
# Usage:
#   ./start_rover_slam.sh                          # fleet size from swarm.yaml
#   ./start_rover_slam.sh max_nb_robots:=2 enable_visualization:=false

export ROS_DOMAIN_ID=43
unset ROS_LOCALHOST_ONLY

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# Kill leftovers from a previous SLAM run only (domain-checked, never global --
# S3E processes on domain 42 must survive).
for pid in $(pgrep -f 'icp_odometry|lidar_handler_node|loop_closure_detection_node|pose_graph_manager|visualization_node|rviz2' 2>/dev/null); do
    if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -q '^ROS_DOMAIN_ID=43$'; then
        kill -9 "$pid" 2>/dev/null
    fi
done
sleep 1

echo "[start_rover_slam] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

exec ros2 launch rover_slam rover_swarm_slam.launch.py "$@"
