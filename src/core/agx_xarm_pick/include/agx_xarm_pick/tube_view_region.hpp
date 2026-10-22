#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace agx_xarm_pick {

struct TopDownViewRegion {
  double x_min_m = 0.0;
  double x_max_m = 0.0;
  double preferred_x_m = 0.0;
  double clearance_min_m = 0.0;
  double clearance_max_m = 0.0;
  double preferred_clearance_m = 0.0;
  double sampling_step_m = 0.0;
  double y_tolerance_m = 0.0;
  std::size_t maximum_candidate_attempts = 0;
};

struct TopDownViewCandidate {
  double tcp_x_m = 0.0;
  double tcp_y_m = 0.0;
  double tcp_z_m = 0.0;
  double preference_cost = 0.0;
};

inline bool valid_top_down_view_region(const TopDownViewRegion& region) {
  const bool finite =
      std::isfinite(region.x_min_m) &&
      std::isfinite(region.x_max_m) &&
      std::isfinite(region.preferred_x_m) &&
      std::isfinite(region.clearance_min_m) &&
      std::isfinite(region.clearance_max_m) &&
      std::isfinite(region.preferred_clearance_m) &&
      std::isfinite(region.sampling_step_m) &&
      std::isfinite(region.y_tolerance_m);
  return finite &&
      region.x_min_m > 0.0 &&
      region.x_min_m < region.x_max_m &&
      region.preferred_x_m >= region.x_min_m &&
      region.preferred_x_m <= region.x_max_m &&
      region.clearance_min_m > 0.0 &&
      region.clearance_min_m < region.clearance_max_m &&
      region.preferred_clearance_m >= region.clearance_min_m &&
      region.preferred_clearance_m <= region.clearance_max_m &&
      region.sampling_step_m > 0.0 &&
      region.y_tolerance_m > 0.0 &&
      region.maximum_candidate_attempts > 0;
}

inline double resolve_runtime_cap_top_z(
    double zero_lift_cap_top_z_m,
    double measured_lift_m) {
  if (!std::isfinite(zero_lift_cap_top_z_m) ||
      !std::isfinite(measured_lift_m) || measured_lift_m < 0.0) {
    throw std::invalid_argument(
        "cap-top height and measured lift must be finite and nonnegative");
  }
  return zero_lift_cap_top_z_m - measured_lift_m;
}

namespace detail {

inline void append_unique_sample(std::vector<double>* samples, double value) {
  constexpr double kDuplicateTolerance = 1e-12;
  const auto duplicate = std::find_if(
      samples->begin(), samples->end(),
      [value](double existing) {
        return std::abs(existing - value) <= kDuplicateTolerance;
      });
  if (duplicate == samples->end()) {
    samples->push_back(value);
  }
}

inline std::vector<double> preference_first_axis_samples(
    double minimum,
    double maximum,
    double preferred,
    double step) {
  std::vector<double> samples;
  append_unique_sample(&samples, preferred);
  for (std::size_t index = 1;; ++index) {
    const double offset = static_cast<double>(index) * step;
    const double lower = preferred - offset;
    const double upper = preferred + offset;
    const bool lower_inside = lower >= minimum;
    const bool upper_inside = upper <= maximum;
    if (!lower_inside && !upper_inside) {
      break;
    }
    if (lower_inside) {
      append_unique_sample(&samples, lower);
    }
    if (upper_inside) {
      append_unique_sample(&samples, upper);
    }
  }
  append_unique_sample(&samples, minimum);
  append_unique_sample(&samples, maximum);
  return samples;
}

}  // namespace detail

inline std::vector<TopDownViewCandidate> sample_top_down_view_candidates(
    const TopDownViewRegion& region,
    double desired_y_m,
    double runtime_cap_top_z_m) {
  if (!valid_top_down_view_region(region) ||
      !std::isfinite(desired_y_m) ||
      !std::isfinite(runtime_cap_top_z_m)) {
    throw std::invalid_argument("top-down view sampling request is invalid");
  }
  const auto x_samples = detail::preference_first_axis_samples(
      region.x_min_m,
      region.x_max_m,
      region.preferred_x_m,
      region.sampling_step_m);
  const auto clearance_samples = detail::preference_first_axis_samples(
      region.clearance_min_m,
      region.clearance_max_m,
      region.preferred_clearance_m,
      region.sampling_step_m);
  std::vector<TopDownViewCandidate> candidates;
  candidates.reserve(x_samples.size() * clearance_samples.size());
  for (double clearance : clearance_samples) {
    for (double x : x_samples) {
      candidates.push_back(
          TopDownViewCandidate{
              x,
              desired_y_m,
              runtime_cap_top_z_m + clearance,
              std::abs(x - region.preferred_x_m) +
                  std::abs(clearance - region.preferred_clearance_m)});
    }
  }
  std::stable_sort(
      candidates.begin(), candidates.end(),
      [&region, runtime_cap_top_z_m](
          const TopDownViewCandidate& left,
          const TopDownViewCandidate& right) {
        return std::make_tuple(
                   left.preference_cost,
                   std::abs(
                       left.tcp_z_m - runtime_cap_top_z_m -
                       region.preferred_clearance_m),
                   std::abs(left.tcp_x_m - region.preferred_x_m),
                   left.tcp_z_m,
                   left.tcp_x_m) <
            std::make_tuple(
                   right.preference_cost,
                   std::abs(
                       right.tcp_z_m - runtime_cap_top_z_m -
                       region.preferred_clearance_m),
                   std::abs(right.tcp_x_m - region.preferred_x_m),
                   right.tcp_z_m,
                   right.tcp_x_m);
      });
  if (candidates.size() > region.maximum_candidate_attempts) {
    candidates.resize(region.maximum_candidate_attempts);
  }
  return candidates;
}

}  // namespace agx_xarm_pick
