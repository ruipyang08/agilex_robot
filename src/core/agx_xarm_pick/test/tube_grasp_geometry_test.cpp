#include "agx_xarm_pick/tube_grasp_geometry.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <optional>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

bool Near(double left, double right, double tolerance = 1e-9) {
  return std::abs(left - right) <= tolerance;
}

bool SameDirection(const agxcore::Point2& left,
                   const agxcore::Point2& right) {
  return Near(left.x, right.x) && Near(left.y, right.y);
}

int Fail(const char* message) {
  std::cerr << message << '\n';
  return EXIT_FAILURE;
}

}  // namespace

int main() {
  using agxcore::GapSample;
  using agxcore::Point2;
  using agxcore::choose_grasp_diagonal;
  using agxcore::command_for_inner_gap;
  using agxcore::diagonal_clearance;
  using agxcore::diagonal_directions;

  const std::vector<GapSample> calibration = {
      {0.0, 0.000},
      {500.0, 0.050},
      {1000.0, 0.100},
  };
  const auto insertion = command_for_inner_gap(calibration, 0.024);
  if (!insertion || !Near(*insertion, 240.0)) {
    return Fail("physical insertion gap was not interpolated");
  }
  if (command_for_inner_gap(calibration, 0.120)) {
    return Fail("gap interpolation extrapolated past calibration");
  }
  if (command_for_inner_gap(calibration, -0.001)) {
    return Fail("gap interpolation extrapolated below calibration");
  }
  if (command_for_inner_gap({{0.0, 0.0}}, 0.0)) {
    return Fail("too few calibration samples were accepted");
  }
  if (command_for_inner_gap(
          {{0.0, 0.000}, {500.0, 0.050}, {400.0, 0.100}}, 0.024) ||
      command_for_inner_gap(
          {{0.0, 0.000}, {500.0, 0.050}, {1000.0, 0.040}}, 0.024)) {
    return Fail("non-monotonic calibration was accepted");
  }
  if (command_for_inner_gap(
          {{0.0, 0.000},
           {500.0, 0.050},
           {std::numeric_limits<double>::infinity(), 0.100}},
          0.024)) {
    return Fail("non-finite trailing calibration was accepted");
  }
  if (command_for_inner_gap(
          calibration, std::numeric_limits<double>::quiet_NaN())) {
    return Fail("non-finite desired gap was accepted");
  }
  const double maximum = std::numeric_limits<double>::max();
  const std::vector<GapSample> extreme_calibration = {
      {-maximum, 0.0},
      {maximum, 1.0},
  };
  const auto low_endpoint =
      command_for_inner_gap(extreme_calibration, 0.0);
  const auto high_endpoint =
      command_for_inner_gap(extreme_calibration, 1.0);
  if (!low_endpoint || *low_endpoint != -maximum ||
      !high_endpoint || *high_endpoint != maximum) {
    return Fail("exact calibration endpoints were not preserved");
  }
  if (command_for_inner_gap(extreme_calibration, 0.5)) {
    return Fail("non-finite interpolation result was accepted");
  }
  if (command_for_inner_gap({{0.0, -0.010}, {100.0, 0.010}}, 0.005)) {
    return Fail("negative physical calibration gap was accepted");
  }

  const double angle = 23.0 * kPi / 180.0;
  const Point2 axis_u{std::cos(angle), std::sin(angle)};
  const Point2 axis_v{-std::sin(angle), std::cos(angle)};
  const auto diagonals = diagonal_directions(axis_u, axis_v);
  if (!diagonals) {
    return Fail("valid right-handed lattice axes were rejected");
  }
  if (!Near(std::hypot((*diagonals)[0].x, (*diagonals)[0].y), 1.0) ||
      !Near(std::hypot((*diagonals)[1].x, (*diagonals)[1].y), 1.0) ||
      !Near((*diagonals)[0].x * (*diagonals)[1].x +
                (*diagonals)[0].y * (*diagonals)[1].y,
            0.0)) {
    return Fail("lattice diagonals are not orthonormal");
  }
  const double inverse_sqrt_two = std::sqrt(0.5);
  if (!SameDirection(
          (*diagonals)[0],
          {inverse_sqrt_two * (axis_u.x + axis_v.x),
           inverse_sqrt_two * (axis_u.y + axis_v.y)}) ||
      !SameDirection(
          (*diagonals)[1],
          {inverse_sqrt_two * (axis_u.x - axis_v.x),
           inverse_sqrt_two * (axis_u.y - axis_v.y)})) {
    return Fail("lattice diagonal order changed");
  }

  const double nan = std::numeric_limits<double>::quiet_NaN();
  if (diagonal_directions({0.0, 0.0}, {0.0, 1.0}) ||
      diagonal_directions({nan, 0.0}, {0.0, 1.0}) ||
      diagonal_directions({1.0, 0.0}, {0.2, std::sqrt(0.96)}) ||
      diagonal_directions({2.0, 0.0}, {0.0, 1.0}) ||
      diagonal_directions({1.0, 0.0}, {0.0, -1.0})) {
    return Fail("invalid lattice axes were accepted");
  }

  const std::vector<Point2> full_grid = {
      {-0.040, -0.040}, {0.000, -0.040}, {0.040, -0.040},
      {-0.040, 0.000},  {0.000, 0.000},  {0.040, 0.000},
      {-0.040, 0.040},  {0.000, 0.040},  {0.040, 0.040},
  };
  if (choose_grasp_diagonal(
          {0.0, 0.0},
          {},
          {1.0, 0.0},
          {0.0, 1.0},
          0.024,
          0.008,
          0.012,
          0.020,
          0.003) ||
      diagonal_clearance(
          {0.0, 0.0},
          {},
          {1.0, 0.0},
          0.024,
          0.008,
          0.012,
          0.020,
          0.003) ||
      choose_grasp_diagonal(
          {0.0, 0.0},
          {{0.040, 0.0}},
          {1.0, 0.0},
          {0.0, 1.0},
          0.024,
          0.008,
          0.012,
          0.020,
          0.003) ||
      diagonal_clearance(
          {0.0, 0.0},
          {{0.040, 0.0}},
          {1.0, 0.0},
          0.024,
          0.008,
          0.012,
          0.020,
          0.003)) {
    return Fail("clearance accepted occupied points without one target");
  }

  if (diagonal_clearance(
          {0.0, 0.0},
          {{0.0, 0.0}},
          {1.0, 0.0},
          0.020,
          0.008,
          0.012,
          0.020,
          0.003) ||
      choose_grasp_diagonal(
          {0.0, 0.0},
          {{0.0, 0.0}},
          {1.0, 0.0},
          {0.0, 1.0},
          0.020,
          0.008,
          0.012,
          0.020,
          0.003) ||
      diagonal_clearance(
          {0.0, 0.0},
          {{0.0, 0.0}},
          {1.0, 0.0},
          0.019,
          0.008,
          0.012,
          0.020,
          0.003) ||
      choose_grasp_diagonal(
          {0.0, 0.0},
          {{0.0, 0.0}},
          {1.0, 0.0},
          {0.0, 1.0},
          0.019,
          0.008,
          0.012,
          0.020,
          0.003)) {
    return Fail("non-inserting gripper aperture was accepted");
  }

  const auto choice = choose_grasp_diagonal(
      {0.0, 0.0},
      full_grid,
      {1.0, 0.0},
      {0.0, 1.0},
      0.024,
      0.008,
      0.012,
      0.020,
      0.003);
  if (!choice || choice->minimum_clearance_m <= 0.0 ||
      !Near(std::abs(choice->direction.x), inverse_sqrt_two) ||
      !Near(std::abs(choice->direction.y), inverse_sqrt_two)) {
    return Fail("full grid did not select a clear diagonal");
  }
  if (!std::isfinite(choice->yaw_rad) || choice->yaw_rad < -kPi ||
      choice->yaw_rad >= kPi) {
    return Fail("selected yaw is outside [-pi, pi)");
  }

  const auto blocked = choose_grasp_diagonal(
      {0.0, 0.0},
      {{0.0, 0.0},
       {0.012, 0.012},
       {-0.012, -0.012},
       {0.012, -0.012},
       {-0.012, 0.012}},
      {1.0, 0.0},
      {0.0, 1.0},
      0.024,
      0.008,
      0.012,
      0.020,
      0.003);
  if (blocked) {
    return Fail("blocked diagonals did not fail closed");
  }

  const auto asymmetric = choose_grasp_diagonal(
      {0.0, 0.0},
      {{0.0, 0.0}, {0.012, 0.012}},
      {1.0, 0.0},
      {0.0, 1.0},
      0.024,
      0.008,
      0.012,
      0.020,
      0.003);
  if (!asymmetric || asymmetric->minimum_clearance_m <= 0.0 ||
      !SameDirection(
          asymmetric->direction,
          {inverse_sqrt_two, -inverse_sqrt_two})) {
    return Fail("clear asymmetric diagonal was not selected");
  }

  for (const std::array<double, 5>& invalid :
       std::vector<std::array<double, 5>>{
           {0.0, 0.008, 0.012, 0.020, 0.003},
           {0.024, 0.0, 0.012, 0.020, 0.003},
           {0.024, 0.008, 0.0, 0.020, 0.003},
           {0.024, 0.008, 0.012, 0.0, 0.003},
           {0.024, 0.008, 0.012, 0.020, 0.0},
           {0.024, 0.008, 0.012, 0.020, nan},
       }) {
    if (choose_grasp_diagonal(
            {0.0, 0.0},
            full_grid,
            {1.0, 0.0},
            {0.0, 1.0},
            invalid[0],
            invalid[1],
            invalid[2],
            invalid[3],
            invalid[4])) {
      return Fail("invalid grasp geometry was accepted");
    }
  }
  if (choose_grasp_diagonal(
          {nan, 0.0},
          full_grid,
          {1.0, 0.0},
          {0.0, 1.0},
          0.024,
          0.008,
          0.012,
          0.020,
          0.003) ||
      choose_grasp_diagonal(
          {0.0, 0.0},
          {{0.0, 0.0}, {nan, 0.0}},
          {1.0, 0.0},
          {0.0, 1.0},
          0.024,
          0.008,
          0.012,
          0.020,
          0.003)) {
    return Fail("non-finite occupied geometry was accepted");
  }
  if (diagonal_clearance(
          {0.0, 0.0},
          full_grid,
          {nan, 0.0},
          0.024,
          0.008,
          0.012,
          0.020,
          0.003)) {
    return Fail("non-finite diagonal direction was accepted");
  }

  const auto too_near = choose_grasp_diagonal(
      {0.0, 0.0},
      {{0.0, 0.0}, {0.5e-6, 0.0}},
      {1.0, 0.0},
      {0.0, 1.0},
      0.024,
      0.008,
      0.012,
      0.020,
      0.003);
  if (too_near) {
    return Fail("distinct near-target point was ignored");
  }
  if (diagonal_clearance(
          {0.0, 0.0},
          {{0.0, 0.0}, {0.5e-6, 0.0}},
          {1.0, 0.0},
          0.024,
          0.008,
          0.012,
          0.020,
          0.003)) {
    return Fail("duplicate target observations were accepted");
  }

  const auto tie_first = choose_grasp_diagonal(
      {0.0, 0.0},
      {{0.0, 0.0}},
      {1.0, 0.0},
      {0.0, 1.0},
      0.024,
      0.008,
      0.012,
      0.020,
      0.003);
  const auto tie_second = choose_grasp_diagonal(
      {0.0, 0.0},
      {{0.0, 0.0}},
      {1.0, 0.0},
      {0.0, 1.0},
      0.024,
      0.008,
      0.012,
      0.020,
      0.003);
  if (!tie_first || !tie_second ||
      !SameDirection(tie_first->direction, tie_second->direction) ||
      !Near(tie_first->yaw_rad, -kPi / 4.0) ||
      !Near(tie_second->yaw_rad, -kPi / 4.0) ||
      !std::isinf(tie_first->minimum_clearance_m)) {
    return Fail("diagonal tie-break was not deterministic");
  }

  std::vector<Point2> noisy_full_grid = full_grid;
  noisy_full_grid[1].x += 0.001;
  const auto near_tie = choose_grasp_diagonal(
      {0.0, 0.0},
      noisy_full_grid,
      {1.0, 0.0},
      {0.0, 1.0},
      0.024,
      0.008,
      0.012,
      0.020,
      0.003);
  if (!near_tie ||
      !SameDirection(
          near_tie->direction,
          {inverse_sqrt_two, -inverse_sqrt_two})) {
    return Fail("sub-margin clearance noise changed the diagonal tie-break");
  }

  const auto rotated_near_tie = choose_grasp_diagonal(
      {0.0, 0.0},
      full_grid,
      {0.0, -1.0},
      {1.0, 0.0},
      0.024,
      0.008,
      0.012,
      0.020,
      0.003);
  if (!rotated_near_tie ||
      !SameDirection(
          rotated_near_tie->direction,
          {inverse_sqrt_two, -inverse_sqrt_two})) {
    return Fail("equivalent lattice labels changed the preferred grasp line");
  }

  return EXIT_SUCCESS;
}
