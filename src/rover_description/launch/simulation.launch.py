"""Multi-robot rover simulation launch.

Fleet size, namespacing, and spawn poses are read from config/swarm.yaml
(`fleet:` block) — edit that file to change how many rovers spawn and where.
Launch args override the YAML when given. Each rover gets:
  - its own robot_state_publisher with frame_prefix=<ns>/
  - its own /<ns>/robot_description topic
  - a ros_gz_sim `create` targeting that topic with -name <ns>
  - its own parameter_bridge for all /<ns>/* sensor + control topics
  - its own sensor_frame_aliases node (10 identity static TFs in one process)
  - its own sonar_to_range node → /<ns>/sonar/*/range

The RViz config is GENERATED at launch time for exactly N rovers (display
group per rover, distinct colors), using the rover_0 group in
rviz/rover.rviz as the template. Spawn 5 rovers → 5 lidar topics + 5 RViz
groups, no manual editing.

Launch args:
  num_rovers:=N            override fleet.num_rovers from swarm.yaml
  world:=<path>            absolute path to a .sdf world (default: crop_field.sdf)
  full_sensors_all:=bool   override fleet.full_sensors_all from swarm.yaml
  teleop:=bool             spawn the fleet-teleop gnome-terminal (default false:
                           containers without gnome-terminal; run
                           `ros2 run rover_description rover_teleop.py --rovers ...`
                           in your own terminal instead)
  estop:=bool              spawn the e-stop gnome-terminal (default false, same reason)
  rviz:=bool               start the generated per-fleet RViz (default false when the
                           SLAM stack brings its own combined RViz)
  headless:=bool           run the Gazebo server only (-s --headless-rendering);
                           sensors still render via EGL on the GPU
"""
import os
import re
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _sanitize_snap_env():
    """Strip snap-injected vars so GUI children survive any terminal.

    VS Code's snap build exports GTK_PATH / GIO_MODULE_DIR / LOCPATH pointing
    into /snap/..., which makes rviz2 die with a libpthread GLIBC symbol
    error and gnome-terminal exit 127 when the sim is launched from its
    integrated terminal. Launching from a plain desktop terminal never hits
    this. Cleaning here means the launch works the same from both.
    """
    for var in ('GTK_PATH', 'GIO_MODULE_DIR', 'GDK_PIXBUF_MODULE_FILE',
                'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE', 'LOCPATH'):
        if '/snap/' in os.environ.get(var, ''):
            del os.environ[var]
    for var in ('LD_LIBRARY_PATH', 'XDG_DATA_DIRS'):
        val = os.environ.get(var)
        if val and '/snap/' in val:
            os.environ[var] = ':'.join(
                p for p in val.split(':') if '/snap/' not in p)


# Fallbacks if the fleet block is missing from swarm.yaml.
FLEET_DEFAULTS = {
    'num_rovers': 1,
    'namespace_prefix': 'rover_',
    'first_index': 0,
    'full_sensors_all': True,
    'spawn_x': 0.0,
    'spawn_y_start': 0.0,
    'spawn_y_spacing': 0.8,
    'spawn_z': 0.15,
    'spawn_yaw': 0.0,
    'spawn_poses': {},
}

# Per-rover accent colors for the generated RViz config ("R; G; B").
# Cycles if N > len(palette).
RVIZ_PALETTE = [
    '255; 255; 0',    # yellow
    '0; 200; 255',    # cyan
    '255; 0; 200',    # magenta
    '0; 255; 0',      # green
    '255; 165; 0',    # orange
    '170; 85; 255',   # violet
    '255; 80; 80',    # red
    '80; 255; 200',   # mint
    '200; 200; 200',  # grey
    '120; 160; 255',  # steel blue
]


def load_fleet_config(swarm_yaml_path):
    """Read the fleet: block from swarm.yaml, filling in defaults."""
    with open(swarm_yaml_path, 'r') as f:
        data = yaml.safe_load(f) or {}
    params = (data.get('fleet') or {}).get('ros__parameters') or {}
    cfg = dict(FLEET_DEFAULTS)
    cfg.update({k: v for k, v in params.items() if v is not None})
    return cfg


# Two rovers closer than this at spawn = inside each other (rover is ~0.4 m).
SPAWN_CLEARANCE = 0.5


def resolve_spawn_poses(cfg, namespaces):
    """List of (x, y, z, yaw), one per rover, in namespace order.

    Explicit spawn_poses entries win. Rovers without one get the line layout
    (spawn_x, spawn_y_start + i*spacing) — but if that spot is within
    SPAWN_CLEARANCE of an already-assigned pose (e.g. an explicit lane pose),
    the fallback keeps stepping along +Y until clear, so mixed explicit +
    fallback configs never stack rovers on top of each other.
    """
    explicit_map = cfg.get('spawn_poses') or {}
    x0 = float(cfg['spawn_x'])
    y0 = float(cfg['spawn_y_start'])
    dy = float(cfg['spawn_y_spacing'])
    z0 = float(cfg['spawn_z'])
    yaw0 = float(cfg['spawn_yaw'])

    poses = []
    for i, ns in enumerate(namespaces):
        explicit = explicit_map.get(ns)
        if explicit is not None:
            vals = list(explicit) + [0.0] * (4 - len(explicit))
            poses.append(tuple(float(v) for v in vals[:4]))
            continue
        y = y0 + i * dy
        while any((x0 - p[0]) ** 2 + (y - p[1]) ** 2 < SPAWN_CLEARANCE ** 2
                  for p in poses):
            y += dy
        poses.append((x0, y, z0, yaw0))
    return poses


def generate_rviz_config(template_path, rovers_modes):
    """Build an N-rover RViz config from the rover_0 group in rover.rviz.

    rovers_modes: list of (namespace, mode) tuples.
    Returns the path of the generated temp file passed to `rviz2 -d`.
    """
    with open(template_path, 'r') as f:
        cfg = yaml.safe_load(f)

    displays = cfg['Visualization Manager']['Displays']
    kept = []       # non-rover displays (Grid, TF, ...)
    template = None
    for d in displays:
        if d.get('Class') == 'rviz_common/Group':
            if template is None:
                template = d      # first group (rover_0) is the template
            continue
        if d.get('Class') == 'rviz_default_plugins/TF':
            # Drop the hardcoded frame list/tree — RViz rediscovers frames live.
            d.pop('Frames', None)
            d.pop('Tree', None)
        kept.append(d)
    if template is None:
        raise RuntimeError(f'no rviz_common/Group template found in {template_path}')

    template_ns = template.get('Name', 'rover_0')
    template_str = yaml.safe_dump(template)

    groups = []
    for i, (ns, mode) in enumerate(rovers_modes):
        group = yaml.safe_load(template_str.replace(template_ns, ns))
        color = RVIZ_PALETTE[i % len(RVIZ_PALETTE)]
        sub_displays = []
        for d in group['Displays']:
            cls = d.get('Class', '')
            if cls == 'rviz_default_plugins/PointCloud2':
                if mode != 'scout':
                    continue      # lite rovers have no lidar
                d['Color'] = color
            elif cls == 'rviz_default_plugins/Odometry':
                d['Shape']['Color'] = color
            elif cls == 'rviz_default_plugins/Range' and mode != 'scout':
                if d.get('Name') != 'Sonar Front':
                    continue      # lite rovers only keep the front sonar
            sub_displays.append(d)
        group['Displays'] = sub_displays
        groups.append(group)

    cfg['Visualization Manager']['Displays'] = kept + groups
    # Saved window state references the 3-rover panel layout; drop it so RViz
    # lays out panels itself for any N.
    cfg.pop('Window Geometry', None)

    fd, out_path = tempfile.mkstemp(prefix='fleet_', suffix='.rviz')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(cfg, f, default_flow_style=False, sort_keys=False)
    return out_path

# Regex that snips the heavy-sensor blocks out of the URDF for follower rovers.
# See urdf/rover.urdf for the __LITE_STRIP_START__ / __LITE_STRIP_END__ markers.
LITE_STRIP_RE = re.compile(
    r'<!-- __LITE_STRIP_START__.*?__LITE_STRIP_END__ -->',
    re.DOTALL,
)


def spawn_rover(namespace, pose, urdf_template, mode='scout'):
    """Return the list of Nodes that bring up ONE rover under `namespace`.

    pose is (x, y, z, yaw) from swarm.yaml (explicit entry or line layout).
    mode='scout' keeps every sensor.
    mode='lite'  removes lidar and 5 of the 6 sonars — leaving IMU + camera +
    front sonar. The URDF markers get stripped and the ROS-side
    bridge/publisher list shrinks to match, so no orphan topics.
    """
    ns_urdf = urdf_template.replace('__NS__', namespace)
    if mode == 'lite':
        ns_urdf = LITE_STRIP_RE.sub('', ns_urdf)
    x, y, z, yaw = pose

    nodes = []

    # Static TF world → <ns>/odom at the rover's spawn pose. This is what
    # gives every rover a common root in RViz — Fixed Frame `world` sees all N.
    # The rover_i/odom → rover_i/base_link chain (from DiffDrive) hangs off this.
    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'world_to_{namespace}_odom',
        arguments=['--x', str(x), '--y', str(y), '--z', '0',
                   '--yaw', str(yaw),
                   '--frame-id', 'world', '--child-frame-id', f'{namespace}/odom'],
        parameters=[{'use_sim_time': True}],
    ))

    # robot_state_publisher: parses URDF, publishes TF + /<ns>/robot_description.
    # frame_prefix makes every emitted TF frame prefixed with `<ns>/`.
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ns_urdf,
            'use_sim_time': True,
            'frame_prefix': f'{namespace}/',
        }],
    ))

    # Spawn the model into Ignition. -topic is relative to the node's namespace.
    nodes.append(Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        name=f'spawn_{namespace}',
        arguments=[
            '-topic', 'robot_description',
            '-name', namespace,
            '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw),
        ],
        output='screen',
    ))

    # Bridge every namespaced topic. `[` = Ignition→ROS, `]` = ROS→Ignition, `@` = bidir.
    # For lite rovers, drop the topics whose sensors were stripped from the URDF —
    # otherwise the bridge would sit waiting on messages that never arrive.
    bridge_args = [
        f'/{namespace}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        f'/{namespace}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        f'/{namespace}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        f'/{namespace}/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        f'/{namespace}/sonar/front@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        # Camera kept for every rover (moved out of the lite-strip block in the URDF).
        f'/{namespace}/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
    ]
    if mode == 'scout':
        bridge_args += [
            f'/{namespace}/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            f'/{namespace}/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            f'/{namespace}/sonar/front_right@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            f'/{namespace}/sonar/front_left@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            f'/{namespace}/sonar/rear@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            f'/{namespace}/sonar/rear_right@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            f'/{namespace}/sonar/rear_left@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        ]
    nodes.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_{namespace}',
        arguments=bridge_args,
        parameters=[{'use_sim_time': True}],
        output='screen',
    ))

    # Sensor-frame aliases (10 identity static TFs, one process).
    nodes.append(Node(
        package='rover_description',
        executable='sensor_frame_aliases.py',
        namespace=namespace,
        name='sensor_frame_aliases',
        parameters=[{'namespace': namespace, 'use_sim_time': True}],
        output='screen',
    ))

    # Sonar LaserScan → Range converter.
    nodes.append(Node(
        package='rover_description',
        executable='sonar_to_range.py',
        namespace=namespace,
        name='sonar_to_range',
        parameters=[{'namespace': namespace, 'use_sim_time': True}],
        output='screen',
    ))

    # Livox CustomMsg publisher — only for scouts. Lite rovers have no lidar,
    # so there's no PointCloud2 to convert.
    if mode == 'scout':
        nodes.append(Node(
            package='rover_description',
            executable='livox_publisher.py',
            name=f'livox_publisher_{namespace}',
            arguments=['-n', namespace],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ))

    # Battery state publisher — sensor_msgs/BatteryState at 1 Hz.
    # Reads battery_publisher block from config/swarm.yaml.
    pkg = get_package_share_directory('rover_description')
    swarm_yaml = os.path.join(pkg, 'config', 'swarm.yaml')
    nodes.append(Node(
        package='rover_description',
        executable='battery_publisher.py',
        name='battery_publisher',
        namespace=namespace,
        arguments=['-n', namespace],
        parameters=[swarm_yaml, {'use_sim_time': True}],
        output='screen',
    ))

    return nodes


def _launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('rover_description')
    urdf_path = os.path.join(pkg, 'urdf', 'rover.urdf')
    with open(urdf_path, 'r') as f:
        urdf_template = f.read()

    swarm_yaml = os.path.join(pkg, 'config', 'swarm.yaml')
    fleet = load_fleet_config(swarm_yaml)

    # Launch args override swarm.yaml when explicitly given.
    num_arg = LaunchConfiguration('num_rovers').perform(context)
    num_rovers = int(num_arg) if num_arg else int(fleet['num_rovers'])
    num_rovers = max(1, min(10, num_rovers))
    full_arg = LaunchConfiguration('full_sensors_all').perform(context).lower()
    if full_arg in ('1', 'true', 'yes'):
        full_all = True
    elif full_arg in ('0', 'false', 'no'):
        full_all = False
    else:
        full_all = bool(fleet['full_sensors_all'])

    prefix = str(fleet['namespace_prefix'])
    first = int(fleet['first_index'])
    rover_ns_list = [f'{prefix}{first + i}' for i in range(num_rovers)]

    actions = []
    poses = resolve_spawn_poses(fleet, rover_ns_list)
    modes = []
    for i, (ns, pose) in enumerate(zip(rover_ns_list, poses)):
        # Scout: full sensor set. Lite: IMU + camera + front sonar only —
        # saves ~80% of the render/sensor load on weak machines.
        mode = 'scout' if (full_all or i == 0) else 'lite'
        print(f'[simulation.launch.py] {ns}: {mode.upper()} @ '
              f'(x={pose[0]}, y={pose[1]}, z={pose[2]}, yaw={pose[3]})')
        actions.extend(spawn_rover(ns, pose, urdf_template, mode=mode))
        modes.append(mode)

    def _flag(name):
        return LaunchConfiguration(name).perform(context).lower() in ('1', 'true', 'yes')

    # RViz config generated for exactly this fleet — one display group per
    # rover, colors from RVIZ_PALETTE, lidar/sonar displays matching mode.
    # Off by default: the SLAM stack (rover_slam) launches its own combined RViz.
    if _flag('rviz'):
        rviz_template = os.path.join(pkg, 'rviz', 'rover.rviz')
        rviz_generated = generate_rviz_config(rviz_template, list(zip(rover_ns_list, modes)))
        print(f'[simulation.launch.py] generated RViz config for {num_rovers} '
              f'rover(s): {rviz_generated}')
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_generated],
            parameters=[{'use_sim_time': True}],
        ))

    # ONE teleop terminal for the whole fleet — digit keys 1..N switch active rover,
    # 'b' broadcasts identical twist to all (formation drive). Professional pattern:
    # one operator, one window, focus-select. Replaces the old N-terminal setup.
    # Spawn poses are passed so 'r' (reset) teleports each rover to its own pose.
    # Off by default: needs gnome-terminal, absent in containers. Run it manually:
    #   ros2 run rover_description rover_teleop.py --rovers rover_0 rover_1 ...
    if _flag('teleop'):
        actions.append(Node(
            package='rover_description',
            executable='rover_teleop.py',
            name='fleet_teleop',
            arguments=['--rovers', *rover_ns_list,
                       '--spawn-poses', *[f'{p[0]},{p[1]}' for p in poses]],
            prefix='gnome-terminal --wait --title="Fleet Teleop" --',
            output='screen',
        ))

    # Dock monitor — dock pose and thresholds come from swarm.yaml.
    actions.append(Node(
        package='rover_description',
        executable='dock_monitor.py',
        name='dock_monitor_0',
        arguments=['--rovers', *rover_ns_list],
        parameters=[swarm_yaml, {'use_sim_time': True}],
        output='screen',
    ))

    # E-stop manager — fleet-wide /emergency_stop topic + keyboard-driven engage/release.
    # Off by default: needs gnome-terminal, absent in containers.
    if _flag('estop'):
        actions.append(Node(
            package='rover_description',
            executable='estop_manager.py',
            name='estop_manager',
            arguments=['--rovers', *rover_ns_list],
            parameters=[swarm_yaml, {'use_sim_time': True}],
            prefix='gnome-terminal --wait --title="E-Stop (press E / R)" --',
            output='screen',
        ))

    # Diagnostics aggregator — /diagnostics at 1 Hz (rqt_diagnostics_viewer / Foxglove).
    actions.append(Node(
        package='rover_description',
        executable='diagnostics_aggregator.py',
        name='diagnostics_aggregator',
        arguments=['--rovers', *rover_ns_list],
        parameters=[{'use_sim_time': True}],
        output='screen',
    ))

    return actions


def generate_launch_description():
    _sanitize_snap_env()
    pkg = get_package_share_directory('rover_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Make Ignition resolve package:// mesh URIs.
    set_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.dirname(pkg) + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
    )

    default_world = os.path.join(pkg, 'worlds', 'crop_field.sdf')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world')]}.items(),
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )
    # Server-only: no GUI process; gpu_lidar/cameras still render via EGL.
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s --headless-rendering ', LaunchConfiguration('world')]
        }.items(),
        condition=IfCondition(LaunchConfiguration('headless')),
    )

    # Global /clock + /tf bridges — shared across all rovers.
    global_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='global_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # On launch shutdown (Ctrl+C in the launch terminal), close every teleop
    # gnome-terminal we opened. Gnome-terminal's CLI process exits immediately
    # after handing the request to gnome-terminal-server, so `pkill` by title
    # finds nothing. Instead: kill the teleop scripts themselves — gnome-terminal
    # was spawned with --wait, so it closes when its child dies.
    kill_teleops = RegisterEventHandler(
        OnShutdown(on_shutdown=[ExecuteProcess(
            cmd=['pkill', '-f', 'rover_teleop.py'],
            output='log',
        )])
    )

    return LaunchDescription([
        DeclareLaunchArgument('num_rovers', default_value='',
                              description='Number of rovers to spawn (1–10). '
                                          'Empty = use fleet.num_rovers from swarm.yaml.'),
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Absolute path to a .sdf world file'),
        DeclareLaunchArgument('full_sensors_all', default_value='',
                              description='true = every rover gets full sensor set '
                                          '(heavy but comparable). false = only the first '
                                          'rover is scout, rest are lite (IMU + front sonar). '
                                          'Empty = use fleet.full_sensors_all from swarm.yaml.'),
        DeclareLaunchArgument('teleop', default_value='false',
                              description='Spawn fleet teleop in a gnome-terminal '
                                          '(needs gnome-terminal; off in containers).'),
        DeclareLaunchArgument('estop', default_value='false',
                              description='Spawn e-stop manager in a gnome-terminal.'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Start the generated fleet RViz (the SLAM '
                                          'stack usually brings its own).'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Gazebo server only, no GUI '
                                          '(-s --headless-rendering).'),
        set_resource_path,
        gz_sim,
        gz_sim_headless,
        global_bridge,
        OpaqueFunction(function=_launch_setup),
        kill_teleops,
    ])
