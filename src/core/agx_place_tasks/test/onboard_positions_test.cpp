#include "agx_place_tasks/onboard_positions.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <string>

#include <yaml-cpp/yaml.h>

using namespace agx_place_tasks;

namespace {

bool Close(double a, double b) { return std::fabs(a - b) < 1e-9; }

template <class F>
void Throws(F&& f) {
  bool thrown = false;
  try { f(); } catch (const std::invalid_argument&) { thrown = true; }
  assert(thrown);
}

const char* kTable = R"(schema_version: 1
green_tube:
  frame: link_base_lift0
  default: near
  approach_z_m: 0.040
  grasp_z_m: -0.105
  tool_rpy_deg: [180.0, 0.0, 1.32]
  tool_yaw_offset_deg: 45.0
  gripper:
    open_before_descent: 750.0
    close_on_cap: 150.0
    held_margin: 60.0
    force: 50.0
    speed: 50.0
  motion:
    velocity_scaling: 0.05
    acceleration_scaling: 0.05
    eef_step_m: 0.005
  positions:
    far:  [0.3270, -0.0830]
    near: {xy: [0.2524, -0.0764], approach_z_m: -0.020}
)";

}  // namespace

int main() {
  // 1. 正常解析：逐点接近高度覆盖全局默认。
  const auto table = ParsePositions(YAML::Load(kTable), "fixture");
  assert(table.positions.size() == 2);
  assert(table.frame == "link_base_lift0");
  assert(Close(table.grasp_z, -0.105));
  assert(Close(table.tool_yaw_offset_deg, 45.0));
  assert(Close(table.velocity_scaling, 0.05));
  assert(Close(table.close_on_cap, 150.0));
  assert(Close(table.held_margin, 60.0));
  const auto* far = table.Find("far");
  assert(far && Close(far->approach_z, 0.040) && Close(far->grasp_z, -0.105));
  const auto* near = table.Find("near");
  assert(near && Close(near->approach_z, -0.020));
  assert(table.Find("missing") == nullptr);

  // 2. 仓库里那份真实配置必须能加载，且六个点位齐全。
  const auto shipped = std::filesystem::path(__FILE__).parent_path().parent_path() /
                       "config/green_tube_grasp.yaml";
  const auto measured = LoadPositions(shipped.string());
  assert(measured.positions.size() == 6);
  assert(measured.default_name == "1");
  assert(measured.Find("3") && Close(measured.Find("3")->x, 0.3270));
  assert(Close(measured.Find("3")->y, -0.0830));
  assert(Close(measured.Find("4")->approach_z, -0.020));
  assert(Close(measured.Find("2")->approach_z, 0.040));

  // 3. 夹不到的输入必须在加载时就失败。
  {
    auto document = YAML::Load(kTable);
    document["green_tube"]["approach_z_m"] = -0.200;   // below grasp_z
    Throws([&] { ParsePositions(document, "fixture"); });
  }
  {
    auto document = YAML::Load(kTable);
    document["green_tube"]["positions"]["far"] =
        YAML::Load("{xy: [0.3270, -0.0830], approach_z_m: -0.200}");
    Throws([&] { ParsePositions(document, "fixture"); });
  }
  {
    auto document = YAML::Load(kTable);
    document["green_tube"]["positions"]["far"]["diagonal"] = 1.0;   // unknown key
    Throws([&] { ParsePositions(document, "fixture"); });
  }
  {
    auto document = YAML::Load(kTable);
    document["green_tube"].remove("frame");
    Throws([&] { ParsePositions(document, "fixture"); });
  }
  {
    auto document = YAML::Load(kTable);
    document["green_tube"].remove("positions");
    Throws([&] { ParsePositions(document, "fixture"); });
  }
  {
    auto document = YAML::Load(kTable);
    document["green_tube"]["default"] = "nowhere";
    Throws([&] { ParsePositions(document, "fixture"); });
  }
  {
    // 超出记录范围的点位属于配置错误，不是"运行时再看"。
    auto document = YAML::Load(kTable);
    document["green_tube"]["positions"]["far"] = YAML::Load("[9.0, 0.0]");
    Throws([&] { ParsePositions(document, "fixture"); });
  }
  {
    auto document = YAML::Load(kTable);
    document["green_tube"]["motion"]["velocity_scaling"] = 3.0;
    Throws([&] { ParsePositions(document, "fixture"); });
  }
  {
    // 被引号包住的数字是字符串，不是测量值。
    std::string text = kTable;
    const std::string target = "grasp_z_m: -0.105";
    text.replace(text.find(target), target.size(), "grasp_z_m: \"-0.105\"");
    Throws([&] { ParsePositions(YAML::Load(text), "fixture"); });
  }
}
