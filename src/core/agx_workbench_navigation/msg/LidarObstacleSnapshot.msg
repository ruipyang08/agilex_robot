# Original cloud stamp; header.seq is this publisher's transport sequence.
# header.frame_id is the capture base_link frame.
std_msgs/Header header
uint32 cloud_sequence
string source_frame
time transform_stamp
# Explicit permission for a zero-stamped lookup result, valid at all times.
bool transform_static
# Original callback receipt on the onboard host's CLOCK_MONOTONIC.
float64 receipt_steady_s
uint8 CLEAR=0
uint8 BLOCKED=1
uint8 STALE=2
uint8 INVALID=3
uint8 state
string reason
string policy_digest
string filter_digest
# odom -> base_link at header.stamp; never a later, moving base pose.
geometry_msgs/PoseStamped capture_pose
# Exact existing height/self filter. Isolated returns are retained.
sensor_msgs/PointCloud2 obstacle_points
