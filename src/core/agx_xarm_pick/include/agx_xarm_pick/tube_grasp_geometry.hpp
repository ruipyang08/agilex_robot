#ifndef AGX_XARM_PICK_TUBE_GRASP_GEOMETRY_HPP_
#define AGX_XARM_PICK_TUBE_GRASP_GEOMETRY_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace agxcore {

struct GapSample {
  double command;
  double inner_gap_m;
};

struct Point2 {
  double x;
  double y;
};

struct DiagonalChoice {
  Point2 direction;
  double yaw_rad;
  double minimum_clearance_m;
};

namespace detail {

constexpr double kAxisTolerance = 1e-9;
constexpr double kMinimumVectorNorm = 1e-12;
constexpr double kTargetCoincidenceToleranceM = 1e-6;
constexpr double kPi = 3.14159265358979323846;

inline bool finite(Point2 point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

inline bool valid_geometry(double inner_gap_m,
                           double finger_thickness_m,
                           double finger_width_m,
                           double cap_diameter_m,
                           double margin_m) {
  return std::isfinite(inner_gap_m) && inner_gap_m > 0.0 &&
         std::isfinite(finger_thickness_m) && finger_thickness_m > 0.0 &&
         std::isfinite(finger_width_m) && finger_width_m > 0.0 &&
         std::isfinite(cap_diameter_m) && cap_diameter_m > 0.0 &&
         inner_gap_m > cap_diameter_m &&
         std::isfinite(margin_m) && margin_m > 0.0;
}

inline std::optional<std::size_t> single_target_index(
    Point2 target,
    const std::vector<Point2>& occupied) {
  if (!finite(target)) {
    return std::nullopt;
  }
  std::size_t match_count = 0;
  std::size_t target_index = 0;
  for (std::size_t index = 0; index < occupied.size(); ++index) {
    if (!finite(occupied[index])) {
      return std::nullopt;
    }
    const double distance =
        std::hypot(occupied[index].x - target.x, occupied[index].y - target.y);
    if (!std::isfinite(distance)) {
      return std::nullopt;
    }
    if (distance < kTargetCoincidenceToleranceM) {
      ++match_count;
      target_index = index;
    }
  }
  if (match_count != 1) {
    return std::nullopt;
  }
  return target_index;
}

inline std::optional<double> diagonal_clearance_with_target(
    Point2 target,
    const std::vector<Point2>& occupied,
    std::size_t target_index,
    Point2 direction,
    double inner_gap_m,
    double finger_thickness_m,
    double finger_width_m,
    double cap_diameter_m,
    double margin_m) {
  const double finger_offset =
      inner_gap_m / 2.0 + finger_thickness_m / 2.0;
  const double finger_radius =
      std::hypot(finger_thickness_m, finger_width_m) / 2.0;
  const double exclusion =
      cap_diameter_m / 2.0 + finger_radius + margin_m;
  if (!std::isfinite(finger_offset) || !std::isfinite(finger_radius) ||
      !std::isfinite(exclusion)) {
    return std::nullopt;
  }

  double clearance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < occupied.size(); ++index) {
    if (index == target_index) {
      continue;
    }
    const Point2& point = occupied[index];
    for (const double sign : {-1.0, 1.0}) {
      const Point2 finger{
          target.x + sign * finger_offset * direction.x,
          target.y + sign * finger_offset * direction.y,
      };
      if (!finite(finger)) {
        return std::nullopt;
      }
      const double center_distance =
          std::hypot(point.x - finger.x, point.y - finger.y);
      if (!std::isfinite(center_distance)) {
        return std::nullopt;
      }
      clearance =
          std::min(clearance, center_distance - exclusion);
    }
  }
  return clearance;
}

inline double normalized_yaw(Point2 direction) {
  double yaw = std::atan2(direction.y, direction.x);
  if (yaw >= kPi) {
    yaw -= 2.0 * kPi;
  } else if (yaw < -kPi) {
    yaw += 2.0 * kPi;
  }
  return yaw;
}

inline double normalized_line_yaw(double yaw_rad) {
  if (yaw_rad >= kPi / 2.0) {
    yaw_rad -= kPi;
  } else if (yaw_rad < -kPi / 2.0) {
    yaw_rad += kPi;
  }
  return yaw_rad;
}

}  // namespace detail

inline std::optional<double> command_for_inner_gap(
    const std::vector<GapSample>& samples,
    double desired_gap_m) {
  if (!std::isfinite(desired_gap_m) || desired_gap_m < 0.0 ||
      samples.size() < 2) {
    return std::nullopt;
  }

  for (const GapSample& sample : samples) {
    if (!std::isfinite(sample.command) ||
        !std::isfinite(sample.inner_gap_m) || sample.inner_gap_m < 0.0) {
      return std::nullopt;
    }
  }
  for (std::size_t index = 1; index < samples.size(); ++index) {
    if (!(samples[index - 1].command < samples[index].command &&
          samples[index - 1].inner_gap_m < samples[index].inner_gap_m)) {
      return std::nullopt;
    }
  }

  if (desired_gap_m < samples.front().inner_gap_m ||
      desired_gap_m > samples.back().inner_gap_m) {
    return std::nullopt;
  }
  for (const GapSample& sample : samples) {
    if (desired_gap_m == sample.inner_gap_m) {
      return sample.command;
    }
  }
  for (std::size_t index = 1; index < samples.size(); ++index) {
    const GapSample& low = samples[index - 1];
    const GapSample& high = samples[index];
    if (desired_gap_m <= high.inner_gap_m) {
      const double fraction =
          (desired_gap_m - low.inner_gap_m) /
          (high.inner_gap_m - low.inner_gap_m);
      const double command =
          low.command + fraction * (high.command - low.command);
      if (!std::isfinite(command)) {
        return std::nullopt;
      }
      return command;
    }
  }
  return std::nullopt;
}

inline std::optional<Point2> normalized(Point2 point) {
  if (!detail::finite(point)) {
    return std::nullopt;
  }
  const double norm = std::hypot(point.x, point.y);
  if (!std::isfinite(norm) || norm <= detail::kMinimumVectorNorm) {
    return std::nullopt;
  }
  return Point2{point.x / norm, point.y / norm};
}

// Lattice axes are required to be unit, orthogonal, and right-handed within a
// tight tolerance because they come from a validated square-lattice fit.
inline std::optional<std::array<Point2, 2>> diagonal_directions(
    Point2 axis_u,
    Point2 axis_v) {
  if (!detail::finite(axis_u) || !detail::finite(axis_v)) {
    return std::nullopt;
  }
  const double norm_u = std::hypot(axis_u.x, axis_u.y);
  const double norm_v = std::hypot(axis_v.x, axis_v.y);
  const double dot = axis_u.x * axis_v.x + axis_u.y * axis_v.y;
  const double determinant = axis_u.x * axis_v.y - axis_u.y * axis_v.x;
  if (!std::isfinite(norm_u) || !std::isfinite(norm_v) ||
      !std::isfinite(dot) || !std::isfinite(determinant) ||
      std::abs(norm_u - 1.0) > detail::kAxisTolerance ||
      std::abs(norm_v - 1.0) > detail::kAxisTolerance ||
      std::abs(dot) > detail::kAxisTolerance ||
      determinant <= 0.0 ||
      std::abs(determinant - 1.0) > detail::kAxisTolerance) {
    return std::nullopt;
  }

  const auto positive = normalized(
      {axis_u.x + axis_v.x, axis_u.y + axis_v.y});
  const auto negative = normalized(
      {axis_u.x - axis_v.x, axis_u.y - axis_v.y});
  if (!positive || !negative) {
    return std::nullopt;
  }
  return std::array<Point2, 2>{*positive, *negative};
}

inline std::optional<double> diagonal_clearance(
    Point2 target,
    const std::vector<Point2>& occupied,
    Point2 direction,
    double inner_gap_m,
    double finger_thickness_m,
    double finger_width_m,
    double cap_diameter_m,
    double margin_m) {
  if (!detail::finite(target) || !detail::finite(direction) ||
      !detail::valid_geometry(
          inner_gap_m,
          finger_thickness_m,
          finger_width_m,
          cap_diameter_m,
          margin_m)) {
    return std::nullopt;
  }
  const auto target_index =
      detail::single_target_index(target, occupied);
  if (!target_index) {
    return std::nullopt;
  }
  const double direction_norm = std::hypot(direction.x, direction.y);
  if (!std::isfinite(direction_norm) ||
      std::abs(direction_norm - 1.0) > detail::kAxisTolerance) {
    return std::nullopt;
  }
  return detail::diagonal_clearance_with_target(
      target,
      occupied,
      *target_index,
      direction,
      inner_gap_m,
      finger_thickness_m,
      finger_width_m,
      cap_diameter_m,
      margin_m);
}

inline std::optional<DiagonalChoice> choose_grasp_diagonal(
    Point2 target,
    const std::vector<Point2>& occupied,
    Point2 axis_u,
    Point2 axis_v,
    double inner_gap_m,
    double finger_thickness_m,
    double finger_width_m,
    double cap_diameter_m,
    double margin_m) {
  if (!detail::finite(target) ||
      !detail::valid_geometry(
          inner_gap_m,
          finger_thickness_m,
          finger_width_m,
          cap_diameter_m,
          margin_m)) {
    return std::nullopt;
  }
  const auto target_index =
      detail::single_target_index(target, occupied);
  if (!target_index) {
    return std::nullopt;
  }
  const auto directions = diagonal_directions(axis_u, axis_v);
  if (!directions) {
    return std::nullopt;
  }

  std::array<double, 2> clearances{};
  std::array<double, 2> yaws{};
  for (std::size_t index = 0; index < directions->size(); ++index) {
    const auto clearance = detail::diagonal_clearance_with_target(
        target,
        occupied,
        *target_index,
        (*directions)[index],
        inner_gap_m,
        finger_thickness_m,
        finger_width_m,
        cap_diameter_m,
        margin_m);
    if (!clearance || std::isnan(*clearance)) {
      return std::nullopt;
    }
    clearances[index] = *clearance;
    yaws[index] = detail::normalized_yaw((*directions)[index]);
    if (!std::isfinite(yaws[index]) || yaws[index] < -detail::kPi ||
        yaws[index] >= detail::kPi) {
      return std::nullopt;
    }
  }

  if (clearances[0] <= 0.0 && clearances[1] <= 0.0) {
    return std::nullopt;
  }

  std::size_t selected = 0;
  if (clearances[0] <= 0.0) {
    selected = 1;
  } else if (clearances[1] > 0.0) {
    const double clearance_difference = clearances[1] - clearances[0];
    const bool effectively_equal =
        clearances[1] == clearances[0] ||
        (std::isfinite(clearance_difference) &&
         std::abs(clearance_difference) <= margin_m);
    if (effectively_equal) {
      if (detail::normalized_line_yaw(yaws[1]) <
          detail::normalized_line_yaw(yaws[0])) {
        selected = 1;
      }
    } else if (clearance_difference > 0.0) {
      selected = 1;
    }
  }
  return DiagonalChoice{
      (*directions)[selected], yaws[selected], clearances[selected]};
}

}  // namespace agxcore

#endif  // AGX_XARM_PICK_TUBE_GRASP_GEOMETRY_HPP_
