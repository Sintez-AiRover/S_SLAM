"""cslam lidar stack for ONE sim rover, in the /r{id} namespace cslam requires.

Why this file exists instead of including cslam_experiments' cslam_lidar
.launch.py directly: cslam HARDCODES the /r{robot_id} namespace pattern for
every inter-robot channel (heartbeats in neighbor_monitor.py, descriptor
requests in global_descriptor_loop_closure_detection.py, optimized-estimate
exchange in decentralized_pgo.cpp). Running the nodes under /rover_N broke
robot-to-robot traffic silently: keyframes flowed but optimization never
aggregated (pose graphs stuck at 1 value). So the cslam nodes live in /r{id}
-- exactly like every upstream experiment -- and REMAPPINGS point their
sensor inputs at the sim's /rover_{id} topics. The three node definitions
below mirror cslam_experiments/launch/cslam/cslam_lidar.launch.py; the cslam
packages themselves are not modified.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_id = int(LaunchConfiguration('robot_id').perform(context))
    sim_ns = LaunchConfiguration('sim_namespace').perform(context)  # /rover_N
    ns = f'/r{robot_id}'

    config = [
        os.path.join(get_package_share_directory('rover_slam'), 'config', ''),
        LaunchConfiguration('config_file'),
    ]
    common_params = [
        LaunchConfiguration('config'), {
            'robot_id': robot_id,
            'max_nb_robots': LaunchConfiguration('max_nb_robots'),
        }
    ]
    # The config subscribes relative "lidar/points"/"odom_icp"; remap those
    # from /r{id}/... onto the sim rover's actual topics.
    sensor_remaps = [
        ('lidar/points', f'{sim_ns}/lidar/points'),
        ('odom_icp', f'{sim_ns}/odom_icp'),
    ]

    return [
        Node(package='cslam',
             executable='loop_closure_detection_node.py',
             name='cslam_loop_closure_detection',
             parameters=common_params,
             namespace=ns),
        Node(package='cslam',
             executable='lidar_handler_node.py',
             name='cslam_map_manager',
             parameters=common_params,
             remappings=sensor_remaps,
             namespace=ns),
        Node(package='cslam',
             executable='pose_graph_manager',
             name='cslam_pose_graph_manager',
             parameters=common_params,
             namespace=ns),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='0'),
        DeclareLaunchArgument('max_nb_robots', default_value='3'),
        DeclareLaunchArgument('sim_namespace', default_value='/rover_0'),
        DeclareLaunchArgument('config_file', default_value='rover_lidar.yaml'),
        DeclareLaunchArgument(
            'config',
            default_value=[
                os.path.join(get_package_share_directory('rover_slam'),
                             'config', ''),
                LaunchConfiguration('config_file')
            ]),
        OpaqueFunction(function=launch_setup),
    ])
