# Aggregated view of every robot's latest RobotStatus. Published by
# swarm_table_node.py (single global instance, subscribes to all robots'
# cslam/swarm_status topics).

std_msgs/Header header
uint32 designated_origin_robot_id
RobotStatus[] robots

# Parallel array to robots[] -- observed fraction of missed status_seq
# increments per robot over the trailing loss_window_sec, computed by
# swarm_table_node.py (the only node that sees every robot's stream and
# can detect a missed sequence number). Feeds the "tolerate <=30% packet
# loss" measurement; this is observability, not loss tolerance itself.
float32[] loss_fraction
