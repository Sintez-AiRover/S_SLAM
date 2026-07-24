# Livox custom scan message. Byte-compatible with livox_ros_driver2/msg/CustomMsg.
# FAST-LIO2, Point-LIO, LIO-SAM (Livox variant) all consume this format.
std_msgs/Header header
uint64 timebase          # nanoseconds — reference time for per-point offsets
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
