#include "agx_xarm_pick/visual_alignment.hpp"

#include <cmath>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace {

using agx_xarm_pick::AlignmentMotionResult;
using agx_xarm_pick::AlignmentMotionStatus;
using agx_xarm_pick::AlignmentPose;
using agx_xarm_pick::AlignmentPoseResult;
using agx_xarm_pick::TargetProviderResult;
using agx_xarm_pick::TargetProviderStatus;
using agx_xarm_pick::VisualAlignmentMotion;
using agx_xarm_pick::VisualTargetProvider;
using agx_xarm_pick::VisualAlignmentConfig;
using agx_xarm_pick::VisualAlignmentStatus;
using agx_xarm_pick::VisualTargetObservation;

int Fail(const std::string& detail) {
  std::cerr << detail << '\n';
  return EXIT_FAILURE;
}

VisualTargetObservation Target(double error_px, std::uint64_t stamp_ns) {
  VisualTargetObservation target;
  target.stamp_ns = stamp_ns;
  target.frame_id = "cam1_color_optical_frame";
  target.reference_u_px = 319.5;
  target.reference_v_px = 239.5;
  target.u_px = target.reference_u_px + error_px;
  target.v_px = target.reference_v_px;
  target.image_width = 640;
  target.image_height = 480;
  return target;
}

TargetProviderResult ProviderResult(
    TargetProviderStatus status,
    double error_px = 0.0,
    std::uint64_t stamp_ns = 0) {
  TargetProviderResult result;
  result.status = status;
  result.detail = "fake provider result";
  if (status == TargetProviderStatus::kSuccess) {
    result.observation = Target(error_px, stamp_ns);
  }
  return result;
}

class FakeProvider final : public VisualTargetProvider {
 public:
  TargetProviderResult Select(std::uint64_t not_before_ns) override {
    not_before.push_back(not_before_ns);
    if (results.empty()) {
      return ProviderResult(TargetProviderStatus::kUnavailable);
    }
    auto result = results.front();
    results.pop_front();
    return result;
  }

  std::deque<TargetProviderResult> results;
  std::vector<std::uint64_t> not_before;
};

class FakeMotion final : public VisualAlignmentMotion {
 public:
  AlignmentPoseResult CurrentPose() override {
    AlignmentPoseResult result;
    result.status = pose_status;
    result.pose = pose;
    result.detail = "fake pose";
    return result;
  }

  AlignmentMotionResult AttemptCorrection(
      const VisualTargetObservation& required_fresh_target,
      double correction_x_m,
      double correction_y_m) override {
    targets.push_back(required_fresh_target);
    corrections.emplace_back(correction_x_m, correction_y_m);
    AlignmentMotionResult result;
    result.status = statuses.empty()
        ? AlignmentMotionStatus::kSuccess
        : statuses.front();
    if (!statuses.empty()) {
      statuses.pop_front();
    }
    result.requested_x_m = correction_x_m;
    result.requested_y_m = correction_y_m;
    result.attempted_x_m = correction_x_m;
    result.attempted_y_m = correction_y_m;
    result.cartesian_fraction =
        result.status == AlignmentMotionStatus::kPlanningIncomplete ? 0.5 : 1.0;
    result.before = pose;
    result.after = pose;
    if (result.status == AlignmentMotionStatus::kSuccess) {
      const auto actual_delta = actual_deltas.empty()
          ? std::pair<double, double>{correction_x_m, correction_y_m}
          : actual_deltas.front();
      if (!actual_deltas.empty()) {
        actual_deltas.pop_front();
      }
      pose.tcp_x_m += actual_delta.first;
      pose.tcp_y_m += actual_delta.second;
      pose.state_stamp_ns += 1;
      result.after = pose;
    }
    result.detail = "fake motion result";
    return result;
  }

  AlignmentPose pose{};
  AlignmentMotionStatus pose_status = AlignmentMotionStatus::kSuccess;
  std::deque<AlignmentMotionStatus> statuses;
  std::deque<std::pair<double, double>> actual_deltas;
  std::vector<VisualTargetObservation> targets;
  std::vector<std::pair<double, double>> corrections;
};

struct Harness {
  Harness() {
    motion.pose.valid = true;
    motion.pose.workspace_valid = true;
    motion.pose.tcp_x_m = 0.55;
    motion.pose.tcp_y_m = 0.0;
    motion.pose.tcp_z_m = 0.35;
    motion.pose.qw = 1.0;
    motion.pose.camera_u_axis = {1.0, 0.0, 0.0};
    motion.pose.camera_v_axis = {0.0, 1.0, 0.0};
    motion.pose.camera_forward_axis = {0.0, 0.0, -1.0};
    motion.pose.downward_error_deg = 0.0;
    motion.pose.state_stamp_ns = 1;
  }

  std::uint64_t Settle() {
    settle_ns += 100;
    return settle_ns;
  }

  FakeProvider provider;
  FakeMotion motion;
  VisualAlignmentConfig config;
  std::uint64_t settle_ns = 1000;
  bool cancelled = false;
};

agx_xarm_pick::VisualAlignmentResult Run(Harness* harness) {
  agx_xarm_pick::VisualTargetAligner aligner(
      harness->config,
      &harness->provider,
      &harness->motion,
      [harness]() { return harness->Settle(); },
      [harness]() { return harness->cancelled; },
      2.0);
  return aligner.Run();
}

void QueueErrors(Harness* harness, const std::vector<double>& errors) {
  std::uint64_t stamp = 2000;
  for (const double error : errors) {
    harness->provider.results.push_back(
        ProviderResult(TargetProviderStatus::kSuccess, error, stamp));
    stamp += 1000;
  }
}

}  // namespace

int main() {
  {
    Harness harness;
    QueueErrors(&harness, {120, 80, 45, 18, 7, 6});
    const auto result = Run(&harness);
    if (result.status != VisualAlignmentStatus::kSuccess ||
        harness.motion.corrections.size() != 4 ||
        result.completed_moves != 4) {
      return Fail("bounded convergence did not require two centered frames");
    }
  }
  {
    Harness harness;
    QueueErrors(&harness, {7, 6});
    const auto result = Run(&harness);
    if (result.status != VisualAlignmentStatus::kSuccess ||
        !harness.motion.corrections.empty()) {
      return Fail("already-centered target caused motion");
    }
  }
  {
    Harness harness;
    harness.provider.results.push_back(
        ProviderResult(TargetProviderStatus::kSuccess, 100, 2000));
    harness.provider.results.push_back(
        ProviderResult(TargetProviderStatus::kSuccess, 80, 2000));
    harness.provider.results.push_back(
        ProviderResult(TargetProviderStatus::kSuccess, 70, 2000));
    harness.provider.results.push_back(
        ProviderResult(TargetProviderStatus::kSuccess, 60, 2000));
    const auto result = Run(&harness);
    if (result.status != VisualAlignmentStatus::kTargetUnavailable ||
        harness.motion.corrections.size() != 1) {
      return Fail("reused target timestamp reached a second motion");
    }
  }
  for (const auto status : {
           TargetProviderStatus::kAbsent,
           TargetProviderStatus::kStale,
           TargetProviderStatus::kInconsistent}) {
    Harness harness;
    for (int retry = 0; retry < 3; ++retry) {
      harness.provider.results.push_back(ProviderResult(status));
    }
    const auto result = Run(&harness);
    if (result.status != VisualAlignmentStatus::kTargetUnavailable ||
        harness.provider.not_before.size() != 3 ||
        !(harness.provider.not_before[0] < harness.provider.not_before[1] &&
          harness.provider.not_before[1] < harness.provider.not_before[2]) ||
        !harness.motion.corrections.empty()) {
      return Fail("stationary target retry policy is not bounded and fresh");
    }
  }
  for (const auto status : {
           TargetProviderStatus::kAmbiguous,
           TargetProviderStatus::kInvalidObservation}) {
    Harness harness;
    harness.provider.results.push_back(ProviderResult(status));
    const auto result = Run(&harness);
    if (result.status != VisualAlignmentStatus::kInvalidTarget ||
        harness.provider.not_before.size() != 1 ||
        !harness.motion.corrections.empty()) {
      return Fail("unsafe target status was retried");
    }
  }
  {
    Harness harness;
    QueueErrors(&harness, {100});
    harness.motion.statuses = {
        AlignmentMotionStatus::kPlanningIncomplete,
        AlignmentMotionStatus::kPlanningIncomplete,
        AlignmentMotionStatus::kPlanningIncomplete,
        AlignmentMotionStatus::kPlanningIncomplete};
    const auto result = Run(&harness);
    if (result.status != VisualAlignmentStatus::kPlanningFailed ||
        harness.motion.corrections.size() != 4 ||
        std::abs(std::hypot(
            harness.motion.corrections[0].first,
            harness.motion.corrections[0].second) - 0.05) > 1e-9 ||
        std::abs(std::hypot(
            harness.motion.corrections[1].first,
            harness.motion.corrections[1].second) - 0.025) > 1e-9 ||
        std::abs(std::hypot(
            harness.motion.corrections[2].first,
            harness.motion.corrections[2].second) - 0.0125) > 1e-9 ||
        std::abs(std::hypot(
            harness.motion.corrections[3].first,
            harness.motion.corrections[3].second) - 0.00625) > 1e-9 ||
        result.completed_moves != 0 ||
        result.events.size() != 6 ||
        result.events[1].status != VisualAlignmentStatus::kPlanningFailed ||
        result.events[2].status != VisualAlignmentStatus::kPlanningFailed ||
        result.events[3].status != VisualAlignmentStatus::kPlanningFailed ||
        result.events[4].status != VisualAlignmentStatus::kPlanningFailed) {
      return Fail(
          "planning retry did not use 50/25/12.5/6.25 mm without motion");
    }
  }
  {
    Harness harness;
    QueueErrors(&harness, {100, 60, 7, 6});
    harness.motion.statuses = {
        AlignmentMotionStatus::kStaleTargetBeforeExecution,
        AlignmentMotionStatus::kSuccess};
    const auto result = Run(&harness);
    if (result.status != VisualAlignmentStatus::kSuccess ||
        harness.motion.corrections.size() != 2 ||
        result.completed_moves != 1 ||
        result.events.size() < 4 ||
        result.events[1].status !=
            VisualAlignmentStatus::kTargetUnavailable ||
        result.events[3].status != VisualAlignmentStatus::kSuccess) {
      return Fail("stale target during planning did not trigger re-observation");
    }
  }
  {
    Harness harness;
    QueueErrors(&harness, {100});
    harness.motion.statuses = {AlignmentMotionStatus::kExecutionFailed};
    const auto result = Run(&harness);
    if (result.status != VisualAlignmentStatus::kExecutionFailed ||
        harness.motion.corrections.size() != 1) {
      return Fail("execution failure was blindly retried");
    }
  }
  {
    Harness harness;
    QueueErrors(&harness, {100, 99.5, 99.0});
    const auto result = Run(&harness);
    if (result.status != VisualAlignmentStatus::kNoImprovement ||
        harness.motion.corrections.size() != 2 ||
        std::abs(std::hypot(
            harness.motion.corrections[1].first,
            harness.motion.corrections[1].second) - 0.025) > 1e-9) {
      return Fail("non-improvement did not halve then stop");
    }
  }
  {
    Harness harness;
    QueueErrors(&harness, {20, 19.5, 5, 4});
    const auto result = Run(&harness);
    if (result.status != VisualAlignmentStatus::kSuccess ||
        harness.motion.corrections.size() != 2 ||
        std::abs(std::hypot(
            harness.motion.corrections[0].first,
            harness.motion.corrections[0].second) - 0.015) > 1e-9 ||
        std::abs(std::hypot(
            harness.motion.corrections[1].first,
            harness.motion.corrections[1].second) - 0.0075) > 1e-9) {
      return Fail("near-center non-improvement did not halve the last move");
    }
  }
  {
    Harness harness;
    QueueErrors(&harness, {100, 99.5, 80, 79.5, 60, 7, 6});
    const auto result = Run(&harness);
    if (result.status != VisualAlignmentStatus::kSuccess) {
      return Fail("real improvement did not reset non-improvement counter");
    }
  }
  {
    Harness harness;
    harness.config.maximum_iterations = 12;
    harness.config.maximum_total_displacement_m = 0.75;
    QueueErrors(
        &harness,
        {200, 198, 196, 194, 192, 190, 188, 186, 184, 182, 180, 178, 176});
    const auto result = Run(&harness);
    if (result.status != VisualAlignmentStatus::kIterationLimit ||
        harness.motion.corrections.size() != 12) {
      return Fail("iteration limit did not stop before command 13");
    }
  }
  {
    Harness harness;
    harness.config.maximum_iterations = 20;
    harness.config.maximum_total_displacement_m = 0.20;
    QueueErrors(&harness, {200, 198, 196, 194, 192, 190, 188, 186, 184});
    const auto result = Run(&harness);
    if (result.status != VisualAlignmentStatus::kDisplacementLimit ||
        harness.motion.corrections.size() != 4) {
      return Fail("displacement budget did not stop before command 5");
    }
  }
  {
    Harness harness;
    harness.motion.pose.workspace_valid = false;
    QueueErrors(&harness, {100});
    if (Run(&harness).status != VisualAlignmentStatus::kWorkspaceRejected ||
        !harness.motion.corrections.empty()) {
      return Fail("workspace rejection reached motion");
    }
  }
  {
    Harness harness;
    harness.motion.pose.valid = false;
    QueueErrors(&harness, {100});
    if (Run(&harness).status != VisualAlignmentStatus::kInvalidPose ||
        !harness.motion.corrections.empty()) {
      return Fail("invalid pose reached motion");
    }
  }
  {
    Harness harness;
    harness.motion.pose_status = AlignmentMotionStatus::kInvalidPose;
    harness.motion.pose.valid = false;
    harness.motion.pose.workspace_valid = false;
    QueueErrors(&harness, {100});
    if (Run(&harness).status != VisualAlignmentStatus::kInvalidPose ||
        !harness.motion.corrections.empty()) {
      return Fail("unavailable measured state was mislabeled as workspace");
    }
  }
  {
    Harness harness;
    harness.motion.pose.camera_forward_axis = {0.0, 0.1, -0.99};
    QueueErrors(&harness, {100});
    if (Run(&harness).status != VisualAlignmentStatus::kOrientationRejected ||
        !harness.motion.corrections.empty()) {
      return Fail("non-downward camera reached motion");
    }
  }
  {
    Harness harness;
    harness.cancelled = true;
    QueueErrors(&harness, {100});
    if (Run(&harness).status != VisualAlignmentStatus::kCancelled ||
        !harness.motion.corrections.empty()) {
      return Fail("cancelled controller reached motion");
    }
  }
  return EXIT_SUCCESS;
}
