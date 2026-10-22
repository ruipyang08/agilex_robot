# Continuous localization across navigation legs

`LocalizationPort` and the ROS-free `LocalizationHealth` maintain current pose
evidence throughout every active mission phase, including final settling. Each
leg starts from the currently localized robot pose. A registered staging pose is
only the destination; it never replaces the robot's measured start pose. Losing
health fails the mission with `LOCALIZATION_UNAVAILABLE` and the ordinary stopped
cleanup path.

The onboard hardware profile uses `/amcl_pose` in `map`, RF2O `/odom` with child
`rslidar`, and the connected `map -> odom -> rslidar -> base_link` TF chain.
Pose and odometry require finite values, normalized orientation, finite covariance
with nonnegative diagonal, positive strictly advancing source stamps, and fresh
receipt and source time. Odometry also requires six finite velocity components.
The default receipt and source leases are each 0.75 s, with 0.05 s permitted future
skew. `~localization_receipt_age_s` and `~localization_source_age_s` must be positive,
finite and at most 5 s; changing them requires qualifying the resulting stop delay.
Receiver receipt time is local monotonic time; no cross-process steady timestamp
is compared.

The pose must remain continuous and agree with the current `map -> rslidar` TF
geometry. Bounds are 0.10 m / 0.10 rad plus elapsed source time at the current
0.50 m/s / 0.80 rad/s limits. A jump outside those bounds invalidates readiness.
These are software evidence checks, not a claim of physical localization accuracy
or a replacement for travel posture, payload clearance and stopping qualification.

An unchanged transform timestamp alone does not mean localization is stale. A
stationary robot can retain exactly the same transform while fresh source pose and
odometry continue to confirm it. The adapter polls the existing TF cache with zero
timeout so a paused ROS clock cannot block steady-clock mission cleanup. Missing
or inconsistent TF fails closed.

AMCL may suppress pose updates below its movement threshold. The hardware adapter
requests `/request_nomotion_update` when pose evidence reaches half its receipt
lease. At most one request is outstanding, with a minimum 0.25 s between attempts.
The mission does not wait for the service, and service success never renews health:
only a subsequent actual `/amcl_pose` callback does. A stuck request therefore
expires pose health without accumulating background calls. This is an AMCL sensor
update request; it does not reseed `/initialpose`.

## Simulator source ownership

Both simulation launch branches select `/workbench_nav/localization_pose` through
`localization_pose_topic` and disable the AMCL refresh service with an empty
`localization_nomotion_service`. `ros1_ground_truth_odometry.py` publishes this
`PoseWithCovarianceStamped` only from an actual Isaac odometry callback and only
when it owns `map -> odom`. It derives the `rslidar` pose using the same rigid
offset and world origin as `/odom` and TF, preserving the original source timestamp
and covariance. There is no timer that renews health from cached pose.

Isaac remains the physical source of truth, and AMCL remains an observer in this
profile. The existing one-time startup `/initialpose` seed is unchanged. Later
workbench legs use live pose/TF; they do not reseed, teleport or infer the start
pose from the previous destination.

## Focused verification

Run `tests/test_workbench_localization_health.py`,
`tests/test_workbench_ground_truth_odometry.py` and `tests/test_workbench_mission.py`.
They cover stale/replayed/future/invalid observations, pose jumps, unchanged
stationary transforms with fresh health, the production adapter's bounded refresh
and nonblocking TF polling, original Isaac timestamp preservation, and localization
loss in every active mission phase. Full ROS1 integration and the loaded two-station
physical simulation remain coordinated qualification gates.
