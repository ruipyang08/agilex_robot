#pragma once

#include <agx_workbench_navigation/motion_route.hpp>
#include <array>
#include <cstddef>
#include <vector>

namespace agx_workbench_navigation {
using MotionPolygon = std::vector<std::array<double, 2>>;
struct MotionGrid {
  std::size_t width=0, height=0;
  double resolution_m=0, origin_x_m=0, origin_y_m=0;
  // Snapshot of original occupancy costs: 254 lethal, 255 unknown, 1..253 soft.
  std::vector<unsigned char> cells;
};
struct MotionGeometryPolicy {
  MotionPolygon footprint{{{-.5,-.33}},{{.5,-.33}},{{.5,.33}},{{-.5,.33}}};
  double hard_padding_m=.25, translation_step_m=.01, rotation_step_rad=.00872664626;
  double integration_error_m=0;
  double soft_clearance_m=.25;
  std::size_t max_samples=100000;
};
enum class MotionCollision { Clear, Occupied, Unknown, MapBoundary, Invalid, SampleLimit };
struct MotionSweepResult {
  MotionCollision status=MotionCollision::Invalid;
  std::size_t samples=0, cell_x=0, cell_y=0;
  MotionPose first_pose;
  double interpolation_padding_m=0;
  bool clear() const { return status==MotionCollision::Clear; }
};
bool ValidMotionPolygon(const MotionPolygon& polygon);
bool ValidMotionGrid(const MotionGrid& grid);
bool MotionPolygonContains(const MotionPolygon& outer, const MotionPolygon& inner);
// Filled cell intersection; read-only inputs. Contact, unknown and outside reject.
MotionSweepResult CheckMotionSweep(const MotionGrid& grid,
    const MotionGeometryPolicy& policy, const std::vector<MotionPose>& poses);
// Repeated queries against an owned immutable map snapshot. Row indexes skip
// free cells only; the same filled-cell polygon test remains authoritative.
class MotionSweepChecker {
 public:
  explicit MotionSweepChecker(const MotionGrid& grid);
  MotionSweepResult Check(const MotionGeometryPolicy& policy,
                          const std::vector<MotionPose>& poses) const;
 private:
  MotionGrid grid_;
  std::vector<std::vector<std::size_t>> occupied_rows_;
  std::vector<std::size_t> occupied_prefix_;
};
}  // namespace agx_workbench_navigation
