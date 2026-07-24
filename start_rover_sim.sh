#!/bin/bash
# AiRover Ignition simulation (rover_description) -- terminal 1 of 2.
#
# Isolated on ROS_DOMAIN_ID=43:
#   - domain 0 has a rogue Gazebo /clock from the LAN (10.1.16.127),
#   - domain 42 belongs to the S3E bag experiments -- never share a domain
#     with them (two /clock publishers = blinking-RViz disaster).
# Do NOT set ROS_LOCALHOST_ONLY=1: CycloneDDS caps ~10 participants/host
# with it, and this stack has far more nodes (S3E lesson).
#
# Usage:
#   ./start_rover_sim.sh                       # fleet size from swarm.yaml (3)
#   ./start_rover_sim.sh num_rovers:=2 headless:=true
#
# Drive the rovers from ANOTHER terminal (also export ROS_DOMAIN_ID=43):
#   ros2 run rover_description rover_teleop.py --rovers rover_0 rover_1 rover_2

export ROS_DOMAIN_ID=43
unset ROS_LOCALHOST_ONLY

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# Render Ignition (GUI + gpu_lidar sensors) on the NVIDIA GPU. Without this
# the GLX/dri3 path fails on this container's display and sensor rendering
# falls back to software -> lidar ~1.8 Hz instead of 5 Hz, RTF ~0.35.
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# Kill leftovers from a previous SIM run only -- match by our domain, never
# global pkill (S3E processes on domain 42 must survive).
for pid in $(pgrep -f 'ign gazebo|parameter_bridge|robot_state_publisher|sensor_frame_aliases|sonar_to_range|livox_publisher|battery_publisher|dock_monitor|diagnostics_aggregator' 2>/dev/null); do
    if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -q '^ROS_DOMAIN_ID=43$'; then
        kill -9 "$pid" 2>/dev/null
    fi
done
sleep 1

echo "[start_rover_sim] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[start_rover_sim] teleop (separate terminal):"
echo "    export ROS_DOMAIN_ID=43 && source /root/ros2_ws/install/setup.bash"
echo "    ros2 run rover_description rover_teleop.py --rovers rover_0 rover_1 rover_2"

exec ros2 launch rover_description simulation.launch.py "$@"
