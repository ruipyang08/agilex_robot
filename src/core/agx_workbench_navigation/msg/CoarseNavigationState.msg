std_msgs/Header header
uint8 FOLLOW_ARC=0
uint8 SPIN_ALIGN=1
uint8 GOAL_ALIGN=2
uint8 BLOCKED=3
uint8 GOAL_REACHED=4
uint8 state
bool arc_collision_free
bool rotation_costmap_clear
bool rotation_lidar_clear
float64 path_heading_error_rad
float64 command_radius_m
string reason
# Operation identity for terminal route events. Diagnostic plan delivery does
# not create a new invocation or grant.
string invocation_id
string route_id
string session_id
string grant_id
string source_goal_id
