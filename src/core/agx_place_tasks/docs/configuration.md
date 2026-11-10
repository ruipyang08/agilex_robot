# place_test_tube — 配置与行为

## 1. 这个任务做什么

把夹爪里已经夹住的一支试管，放进**机器人自己身上的架子**（机载架）的指定槽位。

- 机载架相对机械臂是固定的，所以用**示教好的位姿**，**不做识别、不做视觉对准**。
  （视觉只用于外部架：那个架子的位姿随停车变化。）
- 任务不改变 `grasp_test_tube` 的任何行为，两者在配方里前后拼接使用。

```yaml
schema_version: 2
workflow_id: water_bath
tasks:
  - {id: go_external_rack, type: navigate, target: wb-100}
  - {id: pick_tube,        type: grasp_test_tube, fixture_profile: table_grid_3x3_v1, cap_color: green}
  - {id: place_onboard,    type: place_test_tube, slot: 1}
```

## 2. 机载架 profile（`config/onboard_rack_profiles.yaml`）

当前文件里填的是**实测值**，来源 `agilex_ws/src/tasks/doc/measurements.md` 的 M6
（2026-10-14，末端 `tcp_link`，坐标系 `link_base_lift0`），和
`agx_plane_target/scripts/plane_target_marker.py` 的 `PUT_POINTS` 是同一组六个点。

```yaml
schema_version: 2
onboard_rack_profiles:
  onboard_rack_v1:
    parent_frame: link_base_lift0
    tip_link: tcp_link                   # 坐标对应的末端 link（M6 记录的是 tcp_link）
    tool_rpy_deg: [180.00, 0.00, 1.32]   # 示教时记录的工具姿态（sxyz，度）
    tool_yaw_offset_deg: 45.0           # 下降时绕工具轴额外转 45 度（见下）
    descend_axis: [0.0, 0.0, -1.0]       # 下插方向（单位向量）
    approach_rise_m: 0.150               # 接近点 = 槽中心沿 -descend 抬高 0.150 m
    slots:
      1: {name: green-axis -75 mm, center_m: [0.327302, 0.082902, -0.078002]}
      2: {name: centre,            center_m: [0.329030, 0.007922, -0.078002]}
      # ... 共 6 个
    qualified: true
    evidence_id: measurements.md M6 2026-10-14 link_base_lift0 tcp_link
```

`tip_link` 不能省：M6 的坐标是 `tcp_link` 的位置，而 `tcp_link` 在 `link6` 下方 0.170 m。
按 `link6` 规划会把目标外推 0.170 m，落进腕部奇异（试规划实测：|joint5| 从 0.11 掉到 0.001）。

**为什么是逐点列表而不是“间距 + 轴向”**：实测布局是 3×2 的网格（两个方向各 75 mm），
线性模型描述不了；逐点列表也保证配方里的 `slot: N` 和示教记录里的 N 是同一个 N。

**为什么要 `tool_yaw_offset_deg`**：2026-11-04 低速试跑时，按示教姿态（偏航 1.32 度）
下降，AG95 的指尖扫到了相邻试管的盖子。75 mm 的网格下，把工具绕自身 Z 轴转 45 度，
两指就落在四个盖子的对角空档里。这个值是现场决定的，换架子/换夹爪要重新确认。

规则：

| 项 | 行为 |
|---|---|
| 缺字段 / 多字段 / 字段类型不对（含被引号包住的数字） | 加载即报错 |
| 槽位编号不是 1..N 连续 | 报错（有空洞会让 `slot: N` 两边对不上） |
| 槽名重复 / 中心点非有限数 / 超出 ±2 m | 报错 |
| `descend_axis` 不是单位向量 | 报错 |
| `approach_rise_m` 不在 0.005–0.50 m | 报错 |
| `qualified: false` | 任务在 prepare 阶段就抛错，配方整体不开始 |
| `qualified: true` 但 `evidence_id` 为空或 `unqualified` | 报错 |
| `slot_tolerance_m` 缺失 | 允许（当前**没有**这六个点的重复性实测，所以空着而不是编一个数） |

`SlotCenter(n)` 直接返回示教值；`ApproachPoint(n) = SlotCenter(n) - approach_rise_m * descend_axis`。

## 3. 任务参数

| 参数 | 类型 | 必填 | 说明 |
|---|---|---|---|
| `slot` | 整数 | 是 | 必须是已示教的槽位，未示教在 prepare 阶段拒绝 |
| `rack_profile` | 字符串 | 否 | 若给出，必须等于宿主已安装的 profile id |

## 3.1 宿主如何安装 profile

```xml
<node pkg="agx_robot_workflow" type="run_robot_workflow" name="run_robot_workflow">
  <param name="onboard_rack_profile" value="$(find agx_place_tasks)/config/onboard_rack_profiles.yaml"/>
  <param name="onboard_rack_id" value="onboard_rack_v1"/>
</node>
```

两个参数必须同时给；只给一个、或文件/字段不合法，宿主启动即失败（不降级、不猜默认值）。
都不给时任务类型仍然注册，但 `prepare` 会抛 `no onboard rack profile is installed`。

## 4. 前置条件（全部是实测，不是前序任务自述）

1. 前序 `navigate` 成功，`plan.held_station` 存在；
2. 前序任务可能留下一支被夹住的管（`plan.payload_may_be_retained`）；
3. 底座实测停稳 + 持有 base hold（`BaseReady(true)` + `CheckHeld`）；
4. 夹爪实测为 **Holding** 状态且对象身份已知、无 pending transfer；
5. 已安装且 qualified 的机载架 profile。

## 5. 效果与失败语义

声明的效果只有一个：`robot.object_at_holder`，条件 `VerifiedPhysicalEvent`。

| 执行结果 | 状态库记录 | 任务结果 |
|---|---|---|
| 测得张开 + 测得槽位占用 | `ObjectAtHolder`（holder = FixtureSlot + slot） | 成功（还需 stop 确认） |
| 测得张开，但没有槽位占用测量 | `Released`（无 holder） | 失败：管的位置 **uncertain** |
| 张开不确定 | `GripperUncertain` | 失败：管的位置 uncertain |
| 没有张开 | 不改状态，transfer 保持 pending | 失败（不猜 custody） |

也就是说：**没有传感器证明“管在槽里”，本任务不会声称成功**。
`robot.gripper_empty` 由夹爪状态自身的观测给出，不由本任务声明。

## 5.2 从机载架取管（pick_from_onboard）

`place_test_tube` 的逆操作：把某个已示教槽位里的管取到夹爪里。
配方里它和 `place_test_tube` 共用同一份 profile：

```yaml
tasks:
  - {id: take,  type: pick_from_onboard, slot: 3}   # 取出来，握在手里
  - {id: put,   type: place_test_tube,   slot: 5}   # 放到另一个槽位
```

| 项 | 内容 |
|---|---|
| 输入 | `slot`（必须是已示教的槽位） |
| 入口要求 | 实测夹爪为空 + 已到站 + base hold + 合格 profile |
| 效果 | `robot.gripper_holding`（只在实测夹到东西时产生） |
| 失败 | 空槽 / 没夹到 → 不产生任何效果；夹到但没停稳 → UnconfirmedStop |

动作和放置对称：接近 → 收窄两指 → 笛卡尔直线下降 → 闭合到 200 → **看实测停在哪个位置**
（停在 >80 表示夹到了，闭合到 ≈0 表示空槽）→ 直线退回 → 停稳。

取管之后 `plan.payload_may_be_retained` 变为 true，正好是 `place_test_tube` 的入口条件，
所以 `pick` → `place` 可以直接串在同一个配方里。

## 5.3 机载蓝盒的六个可用点位（2026-11-04 实机验证）

配置在 `config/green_tube_grasp.yaml`，脚本 `scripts/green_tube_cycle.py` 读取：

```bash
rosrun agx_place_tasks green_tube_cycle.py --position blue_back_left --execute
```

| 名称 | XY (m) | 接近高度 | 说明 |
|---|---|---|---|
| green_a | 0.3270, −0.0830 | +0.040 | 前排角落绿管 |
| green_b | 0.3320, −0.0070 | +0.040 | 前排中间绿管 |
| blue_edge | 0.3240, +0.0670 | +0.040 | 前排边缘蓝管 |
| green_back | 0.2524, −0.0034 | +0.040 | 后排中间绿管 |
| blue_back_right | 0.2524, −0.0764 | **−0.020** | 后排右蓝管 |
| blue_back_left | 0.2524, +0.0716 | **−0.020** | 后排左蓝管 |

要点：

- 动作是 **0 度平移到位 → 原地转 45 度 → 直线下降 → 闭合 → 退回 → 放回 → 转回 0 度**；
- 靠近机器人那排（X≈0.25）腕部接近限位，接近高度必须降到 −0.020，否则直线下降只有 76.7%；
- 孔距 37.5 mm、盖径 35 mm，X 方向有效窗口只有几毫米，坐标必须实测，不能推算；
- 闭合到 300 时若实测停在 305 附近表示夹住；停在 300 表示空。

## 5.4 按实测点位取管（`position:`）

除了按示教槽位（`slot:`），`pick_from_onboard` 也可以直接引用实测位置表里的名字：

```yaml
tasks:
  - {id: take, type: pick_from_onboard, position: green_a}
```

宿主需要加载位置表：

```xml
<param name="onboard_positions"
       value="$(find agx_place_tasks)/config/green_tube_grasp.yaml"/>
```

规则：

| 项 | 行为 |
|---|---|
| `slot` 与 `position` 同时给或都不给 | prepare 报错 |
| 名字不在位置表里 | prepare 报错 |
| 宿主没装位置表却用 `position` | prepare 报错 |
| 位置表里 `approach_z_m <= grasp_z_m` | 加载即报错 |

每个点位可以单独覆盖 `approach_z_m`（靠近机器人的一排必须用更低的高度，否则直线下降走不完）。

## 6. 已知缺口（不要当成已完成）

1. **动作已实现，但没有槽位占用传感器**（流程见第 7 节）：
   真实运行会停在"张开后测到夹爪空、但没有传感器证明管在槽里"这一分支并判失败。
2. **槽位占用没有测量源**：没有相机确认“槽里有管/槽是空的”之前，
   真实运行永远走 `Released` 分支并失败。这是有意的 fail-closed。
3. **带载运输未建模**：放下之后，管子固定在车上但不在夹爪里。
   现有运输词表只有“空 / 夹爪里一支”，`robot.payload_secured_in_onboard_fixture`
   还没加，所以 `place` 之后不要紧跟 `navigate` 去接“空车”假设的任务。
4. **travel profile 未标定**：`place` 之所以能 prepare，前提是同一配方里的
   `navigate` 已经拿到合格 profile；那一层目前仍是 `unqualified`。
4.1 **M6 的接近高度有两处记录**：M6 正文写“上方 100 mm”，而实际执行的
   `PUT_APPROACH_RISE = 0.150`。profile 采用 0.150（真机执行值）。重新示教时统一。

## 7. 动作流程

`RunPlaceMotion()`（`agx_place_tasks/place_motion.cpp`）按顺序做五件事，**任一步失败就返回**：

| 步 | 动作 | 判据 |
|---|---|---|
| 1 | 规划到接近点（槽心沿 -descend 抬高 approach_rise_m） | MoveIt 规划 + 执行成功 |
| 2 | **笛卡尔直线**下降到槽心 | `compute_cartesian_path` 覆盖 100%，分片不够就拒绝，不退回关节插值 |
| 3 | 张开夹爪到 1000 | **等夹爪反馈** position >= 1000-100 才算 released；超时记 release_uncertain |
| 4 | **笛卡尔直线**退回接近点 | 同上，100% 才继续 |
| 5 | 停稳判定 | 六个关节速度 <= 0.02 rad/s，连续 3 次采样 |

2026-11-04 实机低速（0.05）试跑 slot 2：45 度偏航下降 + 退回完成，驱动 err=0 / warn=0，
夹爪为空时 release 被正确判为“本来就没夹东西”，不会假报放下。

全程使用 profile 里的 `tool_yaw_offset_deg`（当前 45 度）绕工具轴旋转后的姿态，
接近点和槽心用同一个姿态。只要发生过下降，无论有没有松开，都会先直线退回接近点再返回结果。

- 停止：每步前后检查 stop/cancel；运动中由宿主原有的 stop RPC 停机，函数本身不会越界阻塞。
- 退回失败时 release 仍会记录，但 detail 写明"工具还在槽位"。
- 默认速度/加速度 0.10、插值步长 0.005 m：这段末态距腕部奇异只有 0.01-0.13 rad
  （见 `docs/reachability_check.md`），不能快。

宿主参数（`run_robot_workflow` 私有命名空间）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `place_velocity_scaling` | 0.10 | 关节速度上限比例 |
| `place_acceleration_scaling` | 0.10 | 加速度上限比例 |
| `place_eef_step_m` | 0.005 | 笛卡尔插值步长 |
| `place_gripper_force` / `place_gripper_speed` | 50 / 50 | AG95 张开参数 |
