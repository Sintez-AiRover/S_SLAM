# Per-robot swarm status heartbeat. Published by swarm_status_node.py.
# Distinct from cslam's own internal cslam/heartbeat (std_msgs/UInt32, SLAM
# liveness only) -- this is a richer, application-layer status broadcast.

std_msgs/Header header
uint32 robot_id

# Which robot's frame this robot's pose graph is currently expressed in,
# and whether that equals the swarm's designated origin robot (i.e. this
# robot has merged into the shared global frame). False for every robot
# until its first successful inter-robot loop closure into the origin.
uint32 origin_robot_id
bool in_global_frame

# Current pose (in robot_id's own frame if in_global_frame is false, in
# the shared global frame otherwise).
geometry_msgs/PoseStamped pose

# Whether this robot's own cslam pose_graph_manager heartbeat is fresh
# (see neighbor_management.max_heartbeat_delay_sec).
bool cslam_alive

# STUB FIELD: no real battery source exists anywhere in this workspace
# (bag-replay only, no live robot power telemetry). Always -1.0 (n/a)
# until a real sensor_msgs/BatteryState source is wired up.
float32 battery_pct

uint32 last_keyframe_id
uint32[] connected_robot_ids

# Monotonic counter, incremented every publish. Lets any subscriber detect
# gaps in this robot's status stream and compute observed packet loss
# (see SwarmTable.loss_fraction).
uint32 status_seq
