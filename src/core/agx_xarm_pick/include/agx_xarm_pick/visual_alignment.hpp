#ifndef AGX_XARM_PICK_VISUAL_ALIGNMENT_HPP
#define AGX_XARM_PICK_VISUAL_ALIGNMENT_HPP

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "agx_xarm_pick/visual_alignment_contract.hpp"

namespace agx_xarm_pick {

enum class TargetProviderStatus {
  kSuccess,
  kAbsent,
  kStale,
  kInconsistent,
  kAmbiguous,
  kInvalidRequest,
  kProfileMismatch,
  kInvalidObservation,
  kUnavailable,
  kCancelled,
};

struct TargetProviderResult {
  TargetProviderStatus status = TargetProviderStatus::kUnavailable;
  VisualTargetObservation observation;
  std::string detail;
};

class VisualTargetProvider {
 public:
  virtual ~VisualTargetProvider() = default;
  virtual TargetProviderResult Select(std::uint64_t not_before_ns) = 0;
};

struct AlignmentPose {
  bool valid = false;
  bool workspace_valid = false;
  double tcp_x_m = 0.0;
  double tcp_y_m = 0.0;
  double tcp_z_m = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  Vec3 camera_u_axis;
  Vec3 camera_v_axis;
  Vec3 camera_forward_axis;
  double downward_error_deg = 0.0;
  std::uint64_t state_stamp_ns = 0;
};

enum class AlignmentMotionStatus {
  kSuccess,
  kPlanningIncomplete,
  kStaleTargetBeforeExecution,
  kExecutionFailed,
  kInvalidEndpoint,
  kWorkspaceRejected,
  kInvalidPose,
  kOrientationRejected,
  kCancelled,
};

struct AlignmentPoseResult {
  AlignmentMotionStatus status = AlignmentMotionStatus::kInvalidPose;
  AlignmentPose pose;
  std::string detail;
};

struct AlignmentMotionResult {
  AlignmentMotionStatus status = AlignmentMotionStatus::kExecutionFailed;
  double requested_x_m = 0.0;
  double requested_y_m = 0.0;
  double attempted_x_m = 0.0;
  double attempted_y_m = 0.0;
  double cartesian_fraction = 0.0;
  AlignmentPose before;
  AlignmentPose after;
  std::string detail;
};

class VisualAlignmentMotion {
 public:
  virtual ~VisualAlignmentMotion() = default;
  virtual AlignmentPoseResult CurrentPose() = 0;
  virtual AlignmentMotionResult AttemptCorrection(
      const VisualTargetObservation& required_fresh_target,
      double correction_x_m,
      double correction_y_m) = 0;
};

enum class VisualAlignmentStatus {
  kSuccess,
  kInvalidConfiguration,
  kTargetUnavailable,
  kInvalidTarget,
  kInvalidPose,
  kOrientationRejected,
  kWorkspaceRejected,
  kPlanningFailed,
  kExecutionFailed,
  kInvalidEndpoint,
  kNoImprovement,
  kIterationLimit,
  kDisplacementLimit,
  kCancelled,
  kInternalError,
};

enum class VisualAlignmentEventType {
  kObservation,
  kMotionAttempt,
  kSummary,
};

struct VisualAlignmentEvent {
  VisualAlignmentEventType type = VisualAlignmentEventType::kObservation;
  VisualAlignmentStatus status = VisualAlignmentStatus::kInternalError;
  std::string detail;
  VisualTargetObservation observation;
  AlignmentMotionResult motion;
  double pixel_error_px = 0.0;
  double cumulative_displacement_m = 0.0;
  int completed_moves = 0;
};

struct VisualAlignmentResult {
  VisualAlignmentStatus status = VisualAlignmentStatus::kInternalError;
  std::string detail;
  VisualTargetObservation final_observation;
  AlignmentPose final_pose;
  double final_pixel_error_px = 0.0;
  double u_axis_gain_m_per_px = 0.0;
  double v_axis_gain_m_per_px = 0.0;
  double cumulative_displacement_m = 0.0;
  int completed_moves = 0;
  std::vector<VisualAlignmentEvent> events;
};

class VisualTargetAligner {
 public:
  using SettleCallback = std::function<std::uint64_t()>;
  using CancellationCallback = std::function<bool()>;

  VisualTargetAligner(
      VisualAlignmentConfig config,
      VisualTargetProvider* target_provider,
      VisualAlignmentMotion* motion,
      SettleCallback settle,
      CancellationCallback cancelled,
      double maximum_downward_error_deg);

  VisualAlignmentResult Run();

 private:
  VisualAlignmentResult Finish(
      VisualAlignmentResult result,
      VisualAlignmentStatus status,
      const std::string& detail) const;

  VisualAlignmentConfig config_;
  VisualTargetProvider* target_provider_ = nullptr;
  VisualAlignmentMotion* motion_ = nullptr;
  SettleCallback settle_;
  CancellationCallback cancelled_;
  double maximum_downward_error_deg_ = 0.0;
};

const char* visual_alignment_status_name(VisualAlignmentStatus status);
const char* event_type_name(VisualAlignmentEventType type);
std::string default_visual_alignment_trace_path(
    const std::string& run_type);
bool write_visual_alignment_trace(
    const std::string& path,
    const std::string& run_type,
    const VisualAlignmentResult& result,
    double u_axis_gain_m_per_px,
    double v_axis_gain_m_per_px,
    std::string* error_detail = nullptr);

}  // namespace agx_xarm_pick

#endif  // AGX_XARM_PICK_VISUAL_ALIGNMENT_HPP
