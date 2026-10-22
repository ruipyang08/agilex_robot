#include "agx_xarm_pick/visual_alignment.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <utility>

namespace agx_xarm_pick {
namespace {

bool FinitePose(const AlignmentPose& pose) {
  return pose.valid && pose.state_stamp_ns != 0 &&
      finite(pose.tcp_x_m) && finite(pose.tcp_y_m) &&
      finite(pose.tcp_z_m) && finite(pose.qx) && finite(pose.qy) &&
      finite(pose.qz) && finite(pose.qw) &&
      finite(pose.downward_error_deg);
}

VisualAlignmentStatus PoseFailureStatus(
    const AlignmentPoseResult& pose_result,
    double maximum_downward_error_deg) {
  if (pose_result.status == AlignmentMotionStatus::kCancelled) {
    return VisualAlignmentStatus::kCancelled;
  }
  if (pose_result.status == AlignmentMotionStatus::kWorkspaceRejected) {
    return VisualAlignmentStatus::kWorkspaceRejected;
  }
  if (pose_result.status == AlignmentMotionStatus::kOrientationRejected) {
    return VisualAlignmentStatus::kOrientationRejected;
  }
  if (pose_result.status != AlignmentMotionStatus::kSuccess ||
      !FinitePose(pose_result.pose)) {
    return VisualAlignmentStatus::kInvalidPose;
  }
  if (!pose_result.pose.workspace_valid) {
    return VisualAlignmentStatus::kWorkspaceRejected;
  }
  if (!finite(pose_result.pose.downward_error_deg) ||
      pose_result.pose.downward_error_deg > maximum_downward_error_deg) {
    return VisualAlignmentStatus::kOrientationRejected;
  }
  return VisualAlignmentStatus::kInvalidPose;
}

VisualAlignmentStatus MotionEventStatus(AlignmentMotionStatus status) {
  switch (status) {
    case AlignmentMotionStatus::kSuccess:
      return VisualAlignmentStatus::kSuccess;
    case AlignmentMotionStatus::kPlanningIncomplete:
      return VisualAlignmentStatus::kPlanningFailed;
    case AlignmentMotionStatus::kStaleTargetBeforeExecution:
      return VisualAlignmentStatus::kTargetUnavailable;
    case AlignmentMotionStatus::kExecutionFailed:
      return VisualAlignmentStatus::kExecutionFailed;
    case AlignmentMotionStatus::kInvalidEndpoint:
      return VisualAlignmentStatus::kInvalidEndpoint;
    case AlignmentMotionStatus::kWorkspaceRejected:
      return VisualAlignmentStatus::kWorkspaceRejected;
    case AlignmentMotionStatus::kInvalidPose:
      return VisualAlignmentStatus::kInvalidPose;
    case AlignmentMotionStatus::kOrientationRejected:
      return VisualAlignmentStatus::kOrientationRejected;
    case AlignmentMotionStatus::kCancelled:
      return VisualAlignmentStatus::kCancelled;
  }
  return VisualAlignmentStatus::kInternalError;
}

bool RetriableStationaryStatus(TargetProviderStatus status) {
  return status == TargetProviderStatus::kAbsent ||
      status == TargetProviderStatus::kStale ||
      status == TargetProviderStatus::kInconsistent;
}

std::string EscapeCsv(const std::string& value) {
  if (value.find_first_of(",\"\r\n") == std::string::npos) {
    return value;
  }
  std::string escaped = "\"";
  for (const char character : value) {
    if (character == '\"') {
      escaped += "\"\"";
    } else {
      escaped += character;
    }
  }
  escaped += "\"";
  return escaped;
}

bool InvalidTargetStatus(TargetProviderStatus status) {
  return status == TargetProviderStatus::kAmbiguous ||
      status == TargetProviderStatus::kInvalidRequest ||
      status == TargetProviderStatus::kProfileMismatch ||
      status == TargetProviderStatus::kInvalidObservation;
}

}  // namespace

VisualTargetAligner::VisualTargetAligner(
    VisualAlignmentConfig config,
    VisualTargetProvider* target_provider,
    VisualAlignmentMotion* motion,
    SettleCallback settle,
    CancellationCallback cancelled,
    double maximum_downward_error_deg)
    : config_(std::move(config)),
      target_provider_(target_provider),
      motion_(motion),
      settle_(std::move(settle)),
      cancelled_(std::move(cancelled)),
      maximum_downward_error_deg_(maximum_downward_error_deg) {}

VisualAlignmentResult VisualTargetAligner::Finish(
    VisualAlignmentResult result,
    VisualAlignmentStatus status,
    const std::string& detail) const {
  result.status = status;
  result.detail = detail;
  VisualAlignmentEvent summary;
  summary.type = VisualAlignmentEventType::kSummary;
  summary.status = status;
  summary.detail = detail;
  summary.observation = result.final_observation;
  summary.pixel_error_px = result.final_pixel_error_px;
  summary.cumulative_displacement_m = result.cumulative_displacement_m;
  summary.completed_moves = result.completed_moves;
  result.events.push_back(std::move(summary));
  return result;
}

VisualAlignmentResult VisualTargetAligner::Run() {
  VisualAlignmentResult result;
  result.u_axis_gain_m_per_px = config_.u_axis_gain_m_per_px;
  result.v_axis_gain_m_per_px = config_.v_axis_gain_m_per_px;
  if (!valid_visual_alignment_config(config_) ||
      target_provider_ == nullptr ||
      motion_ == nullptr || !settle_ || !cancelled_ ||
      !finite(maximum_downward_error_deg_) ||
      maximum_downward_error_deg_ < 0.0 ||
      maximum_downward_error_deg_ > 90.0) {
    return Finish(
        std::move(result), VisualAlignmentStatus::kInvalidConfiguration,
        "invalid visual-alignment controller configuration");
  }
  if (cancelled_()) {
    return Finish(
        std::move(result), VisualAlignmentStatus::kCancelled,
        "alignment cancelled before observation");
  }

  auto pose_result = motion_->CurrentPose();
  if (pose_result.status != AlignmentMotionStatus::kSuccess ||
      !FinitePose(pose_result.pose) || !pose_result.pose.workspace_valid ||
      pose_result.pose.downward_error_deg > maximum_downward_error_deg_) {
    const auto status = PoseFailureStatus(
        pose_result, maximum_downward_error_deg_);
    return Finish(std::move(result), status, pose_result.detail);
  }
  result.final_pose = pose_result.pose;

  std::uint64_t previous_not_before_ns = 0;
  std::uint64_t previous_observation_stamp_ns = 0;
  int stationary_retries = 0;
  int centered_observations = 0;
  int insufficient_improvements = 0;
  bool awaiting_improvement = false;
  double previous_error_px = std::numeric_limits<double>::quiet_NaN();
  double correction_cap_m = config_.maximum_step_m;
  double last_correction_m = std::numeric_limits<double>::quiet_NaN();

  while (true) {
    if (cancelled_()) {
      return Finish(
          std::move(result), VisualAlignmentStatus::kCancelled,
          "alignment cancelled");
    }
    const std::uint64_t not_before_ns = settle_();
    if (not_before_ns == 0 || not_before_ns <= previous_not_before_ns) {
      return Finish(
          std::move(result), VisualAlignmentStatus::kInternalError,
          "settle boundary did not advance");
    }
    previous_not_before_ns = not_before_ns;

    const auto provider_result = target_provider_->Select(not_before_ns);
    if (provider_result.status != TargetProviderStatus::kSuccess) {
      VisualAlignmentEvent event;
      event.type = VisualAlignmentEventType::kObservation;
      event.status = RetriableStationaryStatus(provider_result.status) ||
              provider_result.status == TargetProviderStatus::kUnavailable
          ? VisualAlignmentStatus::kTargetUnavailable
          : VisualAlignmentStatus::kInvalidTarget;
      event.detail = provider_result.detail;
      event.cumulative_displacement_m = result.cumulative_displacement_m;
      event.completed_moves = result.completed_moves;
      result.events.push_back(std::move(event));
      if (provider_result.status == TargetProviderStatus::kCancelled) {
        return Finish(
            std::move(result), VisualAlignmentStatus::kCancelled,
            provider_result.detail);
      }
      if (RetriableStationaryStatus(provider_result.status) &&
          stationary_retries < config_.stationary_target_retries) {
        ++stationary_retries;
        continue;
      }
      if (RetriableStationaryStatus(provider_result.status) ||
          provider_result.status == TargetProviderStatus::kUnavailable) {
        return Finish(
            std::move(result), VisualAlignmentStatus::kTargetUnavailable,
            provider_result.detail);
      }
      return Finish(
          std::move(result),
          InvalidTargetStatus(provider_result.status)
              ? VisualAlignmentStatus::kInvalidTarget
              : VisualAlignmentStatus::kInternalError,
          provider_result.detail);
    }

    const auto& observation = provider_result.observation;
    const bool fresh = valid_visual_target_observation(observation) &&
        observation.stamp_ns > not_before_ns &&
        observation.stamp_ns > previous_observation_stamp_ns;
    if (!fresh) {
      VisualAlignmentEvent event;
      event.type = VisualAlignmentEventType::kObservation;
      event.status = VisualAlignmentStatus::kTargetUnavailable;
      event.detail = "target observation is invalid, stale, or reused";
      event.observation = observation;
      event.cumulative_displacement_m = result.cumulative_displacement_m;
      event.completed_moves = result.completed_moves;
      result.events.push_back(std::move(event));
      if (stationary_retries < config_.stationary_target_retries) {
        ++stationary_retries;
        continue;
      }
      return Finish(
          std::move(result), VisualAlignmentStatus::kTargetUnavailable,
          "fresh target retry limit reached");
    }
    stationary_retries = 0;
    previous_observation_stamp_ns = observation.stamp_ns;
    result.final_observation = observation;
    const double error_px = visual_pixel_error(observation);
    result.final_pixel_error_px = error_px;

    VisualAlignmentEvent observation_event;
    observation_event.type = VisualAlignmentEventType::kObservation;
    observation_event.status = VisualAlignmentStatus::kSuccess;
    observation_event.detail = "fresh visual target selected";
    observation_event.observation = observation;
    observation_event.pixel_error_px = error_px;
    observation_event.cumulative_displacement_m =
        result.cumulative_displacement_m;
    observation_event.completed_moves = result.completed_moves;
    result.events.push_back(std::move(observation_event));

    if (awaiting_improvement) {
      const double improvement_px = previous_error_px - error_px;
      if (!finite(improvement_px) ||
          improvement_px < config_.minimum_pixel_error_improvement_px) {
        ++insufficient_improvements;
        correction_cap_m = finite(last_correction_m) &&
                last_correction_m > 0.0
            ? std::max(
                  config_.minimum_retry_step_m,
                  last_correction_m * 0.5)
            : config_.maximum_step_m * 0.5;
        if (insufficient_improvements >= 2) {
          return Finish(
              std::move(result), VisualAlignmentStatus::kNoImprovement,
              "two corrections failed the minimum pixel improvement; "
              "configured image direction or gain is invalid");
        }
      } else {
        insufficient_improvements = 0;
        correction_cap_m = config_.maximum_step_m;
      }
      awaiting_improvement = false;
    }

    if (error_px <= config_.target_center_tolerance_px) {
      ++centered_observations;
      if (centered_observations >=
          config_.required_centered_observations) {
        pose_result = motion_->CurrentPose();
        if (pose_result.status == AlignmentMotionStatus::kSuccess &&
            FinitePose(pose_result.pose)) {
          result.final_pose = pose_result.pose;
        }
        return Finish(
            std::move(result), VisualAlignmentStatus::kSuccess,
            "target centered in required fresh observations");
      }
      continue;
    }
    centered_observations = 0;

    if (result.completed_moves >= config_.maximum_iterations) {
      return Finish(
          std::move(result), VisualAlignmentStatus::kIterationLimit,
          "maximum completed alignment moves reached");
    }
    const double remaining_displacement_m =
        config_.maximum_total_displacement_m -
        result.cumulative_displacement_m;
    if (!finite(remaining_displacement_m) ||
        remaining_displacement_m <= 1e-9) {
      return Finish(
          std::move(result), VisualAlignmentStatus::kDisplacementLimit,
          "maximum cumulative alignment displacement reached");
    }

    pose_result = motion_->CurrentPose();
    if (pose_result.status != AlignmentMotionStatus::kSuccess ||
        !FinitePose(pose_result.pose) || !pose_result.pose.workspace_valid ||
        pose_result.pose.downward_error_deg > maximum_downward_error_deg_) {
      const auto status = PoseFailureStatus(
          pose_result, maximum_downward_error_deg_);
      return Finish(std::move(result), status, pose_result.detail);
    }
    result.final_pose = pose_result.pose;
    const auto axes = projected_image_axes(
        pose_result.pose.camera_u_axis,
        pose_result.pose.camera_v_axis,
        pose_result.pose.camera_forward_axis,
        maximum_downward_error_deg_);
    if (!axes) {
      return Finish(
          std::move(result), VisualAlignmentStatus::kOrientationRejected,
          "camera image axes are not a valid top-down planar basis");
    }
    const auto correction = bounded_visual_correction(
        config_, observation, *axes, remaining_displacement_m,
        correction_cap_m);
    if (!correction) {
      return Finish(
          std::move(result), VisualAlignmentStatus::kDisplacementLimit,
          "no bounded planar correction remains");
    }
    const double original_magnitude_m =
        std::hypot(correction->x_m, correction->y_m);
    if (!finite(original_magnitude_m) || original_magnitude_m <= 1e-12) {
      return Finish(
          std::move(result), VisualAlignmentStatus::kInternalError,
          "non-centered target produced a zero correction");
    }

    bool reobserve = false;
    for (int retry_index = 0;; ++retry_index) {
      const auto retry_magnitude = next_planning_retry_magnitude(
          original_magnitude_m, retry_index,
          config_.minimum_retry_step_m);
      if (!retry_magnitude) {
        return Finish(
            std::move(result), VisualAlignmentStatus::kPlanningFailed,
            "Cartesian correction remained incomplete at minimum retry step");
      }
      const double scale = *retry_magnitude / original_magnitude_m;
      const double retry_x_m = correction->x_m * scale;
      const double retry_y_m = correction->y_m * scale;
      const auto motion_result = motion_->AttemptCorrection(
          observation, retry_x_m, retry_y_m);

      VisualAlignmentEvent motion_event;
      motion_event.type = VisualAlignmentEventType::kMotionAttempt;
      motion_event.status = MotionEventStatus(motion_result.status);
      motion_event.detail = motion_result.detail;
      motion_event.observation = observation;
      motion_event.motion = motion_result;
      motion_event.pixel_error_px = error_px;
      motion_event.cumulative_displacement_m =
          result.cumulative_displacement_m;
      motion_event.completed_moves = result.completed_moves;
      result.events.push_back(std::move(motion_event));

      switch (motion_result.status) {
        case AlignmentMotionStatus::kSuccess: {
          const double executed_magnitude_m = std::hypot(
              motion_result.attempted_x_m,
              motion_result.attempted_y_m);
          if (!finite(executed_magnitude_m) ||
              executed_magnitude_m <= 0.0 ||
              executed_magnitude_m > remaining_displacement_m + 1e-9) {
            return Finish(
                std::move(result), VisualAlignmentStatus::kInvalidEndpoint,
                "motion returned an invalid executed correction magnitude");
          }
          result.cumulative_displacement_m += executed_magnitude_m;
          last_correction_m = executed_magnitude_m;
          ++result.completed_moves;
          result.final_pose = motion_result.after;
          previous_error_px = error_px;
          awaiting_improvement = true;
          reobserve = true;
          break;
        }
        case AlignmentMotionStatus::kPlanningIncomplete:
          continue;
        case AlignmentMotionStatus::kStaleTargetBeforeExecution:
          reobserve = true;
          break;
        case AlignmentMotionStatus::kWorkspaceRejected:
          return Finish(
              std::move(result), VisualAlignmentStatus::kWorkspaceRejected,
              motion_result.detail);
        case AlignmentMotionStatus::kInvalidEndpoint:
          return Finish(
              std::move(result), VisualAlignmentStatus::kInvalidEndpoint,
              motion_result.detail);
        case AlignmentMotionStatus::kInvalidPose:
          return Finish(
              std::move(result), VisualAlignmentStatus::kInvalidPose,
              motion_result.detail);
        case AlignmentMotionStatus::kOrientationRejected:
          return Finish(
              std::move(result), VisualAlignmentStatus::kOrientationRejected,
              motion_result.detail);
        case AlignmentMotionStatus::kCancelled:
          return Finish(
              std::move(result), VisualAlignmentStatus::kCancelled,
              motion_result.detail);
        case AlignmentMotionStatus::kExecutionFailed:
          return Finish(
              std::move(result), VisualAlignmentStatus::kExecutionFailed,
              motion_result.detail);
      }
      break;
    }
    if (!reobserve) {
      return Finish(
          std::move(result), VisualAlignmentStatus::kInternalError,
          "alignment motion did not establish a next observation state");
    }
  }
}


const char* visual_alignment_status_name(VisualAlignmentStatus status) {
  switch (status) {
    case VisualAlignmentStatus::kSuccess:
      return "success";
    case VisualAlignmentStatus::kInvalidConfiguration:
      return "invalid_configuration";
    case VisualAlignmentStatus::kTargetUnavailable:
      return "target_unavailable";
    case VisualAlignmentStatus::kInvalidTarget:
      return "invalid_target";
    case VisualAlignmentStatus::kInvalidPose:
      return "invalid_pose";
    case VisualAlignmentStatus::kOrientationRejected:
      return "orientation_rejected";
    case VisualAlignmentStatus::kWorkspaceRejected:
      return "workspace_rejected";
    case VisualAlignmentStatus::kPlanningFailed:
      return "planning_failed";
    case VisualAlignmentStatus::kExecutionFailed:
      return "execution_failed";
    case VisualAlignmentStatus::kInvalidEndpoint:
      return "invalid_endpoint";
    case VisualAlignmentStatus::kNoImprovement:
      return "no_improvement";
    case VisualAlignmentStatus::kIterationLimit:
      return "iteration_limit";
    case VisualAlignmentStatus::kDisplacementLimit:
      return "displacement_limit";
    case VisualAlignmentStatus::kCancelled:
      return "cancelled";
    case VisualAlignmentStatus::kInternalError:
      return "internal_error";
  }
  return "unknown";
}

const char* event_type_name(VisualAlignmentEventType type) {
  switch (type) {
    case VisualAlignmentEventType::kObservation:
      return "observation";
    case VisualAlignmentEventType::kMotionAttempt:
      return "motion_attempt";
    case VisualAlignmentEventType::kSummary:
      return "summary";
  }
  return "unknown";
}

std::string default_visual_alignment_trace_path(
    const std::string& run_type) {
  const auto stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  std::ostringstream path;
  path << "/tmp/agx_xarm_pick/" << run_type << '-' << stamp_ns << ".csv";
  return path.str();
}

bool write_visual_alignment_trace(
    const std::string& path,
    const std::string& run_type,
    const VisualAlignmentResult& result,
    double u_axis_gain_m_per_px,
    double v_axis_gain_m_per_px,
    std::string* error_detail) {
  const auto fail = [error_detail](const std::string& detail) {
    if (error_detail != nullptr) {
      *error_detail = detail;
    }
    return false;
  };
  if (path.empty() || run_type.empty() ||
      !finite(u_axis_gain_m_per_px) || !finite(v_axis_gain_m_per_px)) {
    return fail("trace request is invalid");
  }
  const auto summary_count = std::count_if(
      result.events.begin(), result.events.end(),
      [](const VisualAlignmentEvent& event) {
        return event.type == VisualAlignmentEventType::kSummary;
      });
  if (summary_count != 1 || result.events.empty() ||
      result.events.back().type != VisualAlignmentEventType::kSummary) {
    return fail("alignment result requires exactly one final summary event");
  }
  try {
    const std::filesystem::path output(path);
    if (output.has_parent_path()) {
      std::filesystem::create_directories(output.parent_path());
    }
    std::ofstream stream(output, std::ios::out | std::ios::trunc);
    if (!stream) {
      return fail("could not open alignment trace");
    }
    stream << "schema_version,run_type,event_index,event_type,status,detail,stamp_ns,frame_id,target_u_px,target_v_px,reference_u_px,reference_v_px,pixel_error_px,requested_dx_m,requested_dy_m,attempted_dx_m,attempted_dy_m,cartesian_fraction,before_tcp_x_m,before_tcp_y_m,before_tcp_z_m,before_tcp_qx,before_tcp_qy,before_tcp_qz,before_tcp_qw,before_downward_error_deg,after_tcp_x_m,after_tcp_y_m,after_tcp_z_m,after_tcp_qx,after_tcp_qy,after_tcp_qz,after_tcp_qw,after_downward_error_deg,cumulative_displacement_m,completed_moves,u_axis_gain_m_per_px,v_axis_gain_m_per_px\n";
    stream << std::setprecision(17);
    for (std::size_t index = 0; index < result.events.size(); ++index) {
      const auto& event = result.events[index];
      const auto& observation = event.observation;
      const auto& motion = event.motion;
      stream << 2 << ','
             << EscapeCsv(run_type) << ','
             << index << ','
             << event_type_name(event.type) << ','
             << visual_alignment_status_name(event.status) << ','
             << EscapeCsv(event.detail) << ','
             << observation.stamp_ns << ','
             << EscapeCsv(observation.frame_id) << ','
             << observation.u_px << ','
             << observation.v_px << ','
             << observation.reference_u_px << ','
             << observation.reference_v_px << ','
             << event.pixel_error_px << ','
             << motion.requested_x_m << ','
             << motion.requested_y_m << ','
             << motion.attempted_x_m << ','
             << motion.attempted_y_m << ','
             << motion.cartesian_fraction << ','
             << motion.before.tcp_x_m << ','
             << motion.before.tcp_y_m << ','
             << motion.before.tcp_z_m << ','
             << motion.before.qx << ','
             << motion.before.qy << ','
             << motion.before.qz << ','
             << motion.before.qw << ','
             << motion.before.downward_error_deg << ','
             << motion.after.tcp_x_m << ','
             << motion.after.tcp_y_m << ','
             << motion.after.tcp_z_m << ','
             << motion.after.qx << ','
             << motion.after.qy << ','
             << motion.after.qz << ','
             << motion.after.qw << ','
             << motion.after.downward_error_deg << ','
             << event.cumulative_displacement_m << ','
             << event.completed_moves << ','
             << u_axis_gain_m_per_px << ','
             << v_axis_gain_m_per_px << '\n';
    }
    if (!stream) {
      return fail("failed while writing alignment trace");
    }
  } catch (const std::exception& error) {
    return fail(error.what());
  }
  if (error_detail != nullptr) {
    error_detail->clear();
  }
  return true;
}

}  // namespace agx_xarm_pick
