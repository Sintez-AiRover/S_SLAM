"""Swarm-SLAM on the AiRover Ignition simulation fleet.

Run order (separate terminals / scripts, all with the same ROS_DOMAIN_ID):

    1. ros2 launch rover_description simulation.launch.py       (the sim)
    2. ros2 launch rover_slam rover_swarm_slam.launch.py        (this file)
    3. ros2 run rover_description rover_teleop.py --rovers rover_0 rover_1 rover_2

Per rover i this starts:
  - rtabmap icp_odometry in /rover_i  (rtabmap_rover_lidar_odometry.launch.py)
  - cslam lidar frontend+backend in /r{i} -- cslam hardcodes that namespace
    for inter-robot topics; inputs remapped to /rover_i
    (cslam_rover_lidar.launch.py; cslam packages never modified)
plus, once, behind enable_visualization:
  - cslam_visualization + RViz     (upstream lidar viz: pose-graph markers on
                                    /cslam/viz/pose_graph_markers, keyframe cloud
                                    markers on /cslam/viz/cloudmarker,
                                    fixed frame robot0_map)

Fleet size defaults to the `fleet:` block of rover_description/config/swarm.yaml
(same source of truth the sim launch uses), overridable with max_nb_robots:=N.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, PopLaunchConfigurations,
                            PushLaunchConfigurations, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def _fleet_from_swarm_yaml():
    """(num_rovers, ns_prefix, first_index) from rover_description's swarm.yaml."""
    path = os.path.join(get_package_share_directory('rover_description'),
                        'config', 'swarm.yaml')
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
        params = (data.get('fleet') or {}).get('ros__parameters') or {}
        return (int(params.get('num_rovers', 3)),
                str(params.get('namespace_prefix', 'rover_')),
                int(params.get('first_index', 0)))
    except (OSError, ValueError, yaml.YAMLError):
        return 3, 'rover_', 0


def launch_setup(context, *args, **kwargs):
    yaml_n, ns_prefix, first = _fleet_from_swarm_yaml()
    override = LaunchConfiguration('max_nb_robots').perform(context)
    max_nb_robots = int(override) if override not in ('', 'from_yaml') else yaml_n

    rover_slam_share = get_package_share_directory('rover_slam')

    actions = [SetParameter(name='use_sim_time', value=True)]

    for i in range(first, first + max_nb_robots):
        ns = f'/{ns_prefix}{i}'

        # Each robot's map frame IS its odometry origin (identity); PGO
        # re-parents robotN_map under robot0_map when robots merge. This
        # also puts rover_N/odom_icp into TF (icp_odometry itself runs with
        # publish_tf=false to stay out of DiffDrive's odom tree).
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'robot{i}_map_to_odom_icp',
            arguments=['--frame-id', f'robot{i}_map',
                       '--child-frame-id', f'{ns_prefix}{i}/odom_icp'],
            parameters=[{'use_sim_time': True}],
        ))

        # ICP odometry (publishes /<ns>/odom_icp, no TF). Every include is
        # wrapped in Push/PopLaunchConfigurations: without it, launch
        # configurations DECLARED INSIDE one include (e.g. 'config') leak
        # into the next include's defaults -- that leak once handed the
        # visualization node rover_lidar.yaml instead of its own config
        # (same bug the S3E launch guards against).
        actions.append(PushLaunchConfigurations())
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                rover_slam_share, 'launch',
                'rtabmap_rover_lidar_odometry.launch.py')),
            launch_arguments={
                'namespace': ns,
                'robot_id': str(i),
                'log_level': LaunchConfiguration('odom_log_level'),
            }.items(),
        ))
        actions.append(PopLaunchConfigurations())

        # cslam stack (loop closure detection + lidar map manager + PGO).
        # Lives in /r{i} -- cslam hardcodes that namespace pattern for all
        # inter-robot topics -- with its sensor inputs remapped to /rover_i
        # (see cslam_rover_lidar.launch.py).
        actions.append(PushLaunchConfigurations())
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                rover_slam_share, 'launch', 'cslam_rover_lidar.launch.py')),
            launch_arguments={
                'sim_namespace': ns,
                'robot_id': str(i),
                'max_nb_robots': str(max_nb_robots),
                'config_file': LaunchConfiguration('config_file'),
            }.items(),
        ))
        actions.append(PopLaunchConfigurations())

    # cslam broadcasts robotN_map frames only AFTER an inter-robot merge
    # (upstream design), so nothing anchors the SLAM frames to the sim's TF
    # tree at startup and RViz would show a blank view ("robot0_map does not
    # exist"). Anchor robot0_map to world once, identity. Assumption: origin
    # election picks the lowest robot id (robot 0) -- always true here since
    # all robots are connected from t=0 on one machine. Other robots' map
    # frames join under robot0_map when PGO broadcasts the merge transforms.
    actions.append(Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='world_to_robot0_map',
        arguments=['--frame-id', 'world', '--child-frame-id', 'robot0_map'],
        parameters=[{'use_sim_time': True}],
    ))

    # Visualization: upstream cslam_visualization lidar launch, unmodified,
    # with rover_slam's own RViz layout (fixed frame world).
    viz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('cslam_visualization'),
            'launch', 'visualization_lidar.launch.py')),
        launch_arguments={
            'config_path': os.path.join(
                get_package_share_directory('cslam_visualization'), 'config/'),
            'config_file': 'lidar.yaml',
            'rviz_config': os.path.join(rover_slam_share, 'config',
                                        'rover_swarm.rviz'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_visualization')),
    )
    # RViz on the NVIDIA GPU (PRIME offload) -- llvmpipe is ~1 fps here.
    actions.append(SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'))
    actions.append(SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'))
    actions.append(PushLaunchConfigurations())
    actions.append(viz)
    actions.append(PopLaunchConfigurations())

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'max_nb_robots', default_value='from_yaml',
            description='Fleet size; default = fleet.num_rovers in '
                        'rover_description/config/swarm.yaml.'),
        DeclareLaunchArgument('config_file', default_value='rover_lidar.yaml'),
        DeclareLaunchArgument('enable_visualization', default_value='true'),
        DeclareLaunchArgument('odom_log_level', default_value='warn'),
        OpaqueFunction(function=launch_setup),
    ])
