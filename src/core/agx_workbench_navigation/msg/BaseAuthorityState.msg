std_msgs/Header header
uint8 NONE=0
uint8 NAVIGATION=1
uint8 STOPPING_FOR_HANDOVER=2
uint8 DOCKING=3
uint8 FAULT=4
uint8 state
string selected_source
string reason
bool manipulation_hold_active
# Boot identity and monotonic generation for every hold/stop/fault assertion.
string session_id
uint64 hold_epoch
string grant_id
string source_goal_id
# Canonical signed base motion. State heartbeats never renew this observation.
# motion_stamp is the original pose endpoint; age is since its original local
# receipt, measured at this state publication. Limits are immutable per session.
bool motion_valid
uint64 motion_revision
time motion_stamp
string motion_frame_id
geometry_msgs/Twist motion
float64 motion_receipt_age_s
float64 motion_receipt_limit_s
float64 motion_header_limit_s
float64 motion_future_tolerance_s
float64 linear_progress_lead_mps
float64 angular_progress_lead_radps
