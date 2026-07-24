"""Per-rover rtabmap ICP odometry for the AiRover Ignition sim.

Simplified from cslam_experiments/launch/odometry/rtabmap_s3e_lidar_odometry
.launch.py (which is left untouched). One icp_odometry node per rover:

  input : /<ns>/lidar/points   (PointCloud2 from the ros_gz bridge, 16x360 @ 5 Hz)
  output: /<ns>/odom_icp       (nav_msgs/Odometry, consumed by cslam's
                                lidar_handler via frontend.odom_topic)

Design constraints (do not "fix" these):
  - publish_tf is FALSE: the Ignition DiffDrive plugin already broadcasts
    <ns>/odom -> <ns>/base_link. A second TF parent for base_link would make
    the trees fight. cslam only needs the Odometry *message*.
  - The odom topic is remapped to odom_icp so the sim's wheel odometry on
    /<ns>/odom stays untouched (still available as a GT-ish reference).
  - guess_frame_id uses the wheel-odom TF as ICP motion prior: reads TF only,
    publishes nothing, big robustness win on sparse clouds.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration('namespace').perform(context)      # e.g. /rover_0
    robot = ns.strip('/')                                       # e.g. rover_0
    use_wheel_guess = LaunchConfiguration('use_wheel_odom_guess').perform(
        context).lower() in ('1', 'true', 'yes')

    return [
        SetParameter(name='use_sim_time', value=True),
        Node(
            package='rtabmap_odom', executable='icp_odometry',
            name='icp_odometry', namespace=ns, output='screen',
            parameters=[{
                'frame_id': f'{robot}/base_link',
                'odom_frame_id': f'{robot}/odom_icp',
                'publish_tf': False,
                'wait_for_transform': 0.3,
                'wait_imu_to_init': False,
                'approx_sync': True,
                'queue_size': 20,
                'qos': 1,  # bridge publishes RELIABLE
                'guess_frame_id': f'{robot}/odom' if use_wheel_guess else '',
                # ICP tuned for a sparse 16-beam, 5 Hz cloud in a ~20 m world
                # of REPEATING corn cylinders. With a good wheel-odom guess the
                # per-frame correction is centimeters; a correspondence radius
                # near the row period (~1 m) let ICP alias one cylinder over
                # (constant 2 m/31 deg failures, odometry lost for good).
                # Keep both radii well under half the row spacing.
                'Icp/VoxelSize': '0.15',
                'Icp/MaxCorrespondenceDistance': '0.4',
                'Icp/PointToPlaneK': '10',
                'Icp/MaxTranslation': '0.5',
                # If registration still fails, re-anchor from the wheel-odom
                # guess on the next frame instead of staying lost forever.
                'Odom/ResetCountdown': '1',
                'Odom/Strategy': '0',
                'OdomF2M/ScanSubtractRadius': '0.15',
                'OdomF2M/ScanMaxSize': '15000',
            }],
            remappings=[
                ('scan_cloud', 'lidar/points'),
                ('odom', 'odom_icp'),
                ('imu', 'imu'),
            ],
            arguments=[
                '--ros-args', '--log-level',
                LaunchConfiguration('log_level').perform(context),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/rover_0'),
        DeclareLaunchArgument('robot_id', default_value='0'),
        DeclareLaunchArgument('use_wheel_odom_guess', default_value='true',
                              description='Use the DiffDrive odom TF as ICP '
                                          'motion prior (read-only).'),
        DeclareLaunchArgument('log_level', default_value='warn'),
        OpaqueFunction(function=launch_setup),
    ])
