# A route is a planning candidate, never an actuator grant.
# header.stamp is response creation time, not a sensor-source freshness stamp.
std_msgs/Header header
string route_id
string invocation_id
string model_digest
string map_digest
string registry_digest
geometry_msgs/Pose2D measured_start
geometry_msgs/Pose2D coarse_start
geometry_msgs/Pose2D goal
PlannedMotionSegment[] segments
float64 estimated_time_s
string departure_source_id
