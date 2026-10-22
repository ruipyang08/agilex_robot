#pragma once
#include <agx_workbench_navigation/motion_preflight.hpp>

namespace agx_workbench_navigation {
struct InstalledMotionPolicy {
  MotionPreflightConfig preflight;
  std::string source_path;
  bool execution_enabled=false;
};
// Exact source-byte identity; independent of preview tools' semantic digest.
std::string MotionBytesDigest(const std::string& bytes);
std::string MotionFileDigest(const std::string& path);
// Strict installed policies. Experimental execution must also be gated by
// simulation mode and live operation/route binding at each actuator boundary.
InstalledMotionPolicy LoadInstalledMotionPolicy(const std::string& path);
}  // namespace agx_workbench_navigation
