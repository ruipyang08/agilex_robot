# 机械臂设备舱上方安全区 · 一期实施规划

> 状态：**待评审/待实施**（2026-09-25 定稿，尚未动代码）
> 本文档可修改；每次改设计请同步更新对应章节。

## 0. 已确认的设计决策

| 项 | 结论 |
|---|---|
| 保护目标 | 防撞舱沿 + 防越出舱口（双目标） |
| 生效范围 | 仅 task 代码；task 入口 enable、出口 disable；RViz/滑条不管 |
| 区域定义 | YAML 手写 box 起步，box_link 系，每条带 `frame_id`+`margin`；保留标定扩展口（二期） |
| 区域数值 | x[-0.396, 0.331]、y[-0.200, 0.201]、z[0.45, ∞)，margin 5cm（舱口投影内缩） |
| 几何判定 | link 碰撞几何 AABB 判包含（不用包围球，不做精确 mesh） |
| 升降 | **入模当第 7 变量**；规划组 `xarm6`/`lifting`/`xarm7` 三组并存 |
| 强制机制 | 规划后轨迹逐点校验为主；planning scene 墙（二期，连带拦 RViz/滑条，已接受） |
| 违规处理 | 拒绝执行 + 重规划 N 次 → 仍失败中止；运行监控（二期）原地保持 + 报错 |
| 入场检查 | 起点在区外 → 拒绝启动（smach 走失败分支） |
| 抓取物 | 不算防护体；不挂 attached object |
| 自碰撞 | 打开 SRDF 中 box_link↔link1/link2（现状被禁用，是盲区） |
| 错误码 | 区分：起点在区外 / 目标在区外 / 轨迹中段越界 / 普通规划失败 |
| 载体 | 新任务（`src/tasks/` 下新包）；校验逻辑抽成独立模块两边共用 |

### 关键实测事实（设计依据）

- 舱体 `2431_box_link.STL`：0.827×0.502×0.581 m，box_link 系 x[-0.446, 0.381]、y[±0.250]、z[-0.011, 0.570]；主顶面 ≈0.39（雷达座 0.393），臂基座 xy 周围有安装塔升到 0.570
- HOME 的 TCP 在 box_link 系 (0.288, 0, 0.840)，距舱前沿 x=0.381 仅 9.3cm → **margin 硬约束 ≤9.3cm**
- 臂可达 0.7m ≫ 舱口半宽 0.25m → 约束真实咬合
- 升降柱**完全不在模型里**：`dummy_base_link→base_link→box_link→arm_base→link_base` 全 fixed，无动态 robot_description、无升降 TF
- `demo_pick` 的 `lift_height_` 是机械臂笛卡尔抬升（`agx_xarm_pick_node.cpp:253-284`），与升降硬件无关
- SRDF 现状：`box_link`↔link3..6+夹爪自碰撞已启用；`box_link`↔link1/link2 被禁用（`ranger_with_xarm6.srdf:90-91`）
- 速度现状：`MoveToA()`/`MoveToCatchPose()` 无速度缩放；`joint_limits.yaml` 3.14 rad/s、无加速度限制
- 升降接口：`/lifter_1/LiftMotorStatePub`（50Hz，`backHeight` 单位 mm，含 `initState`/`reachTargetPos`/限位/故障位）；`/lifter_1/LiftingMotorService`（`val` mm + `mode`，mode=0 位置控制）；调用范例 `joint_slider_panel.cpp:428-459`
- 升降入模既定方案（`decisions.md:143-149`）：插入新关节 `box_link →(fixed)→ lift_column_link →(prismatic 0~0.6, z轴)→ arm_base`，不是改 fixed 关节

## 1. 升降入模（第 7 变量）

### 模型（`ranger_xarm_description/ranger_xarm6_lifting_2431.xacro:81-91`）

现有 `ranger_with_xarm`（fixed，box_link→arm_base，z=0.555）替换为两段：

- `box_link →(fixed)→ lift_column_link`，z = `0.555 − H_calib/1000`
- `lift_column_link →(prismatic lifting_joint, z轴, 0~0.6)→ arm_base`，关节值 = `backHeight/1000`（米）

要点：

- `lift_column_link` 为**空 link**（塔留在 box mesh，接受降低方向保守幻影；若低高度作业被误杀再升级为柱体单独建几何）
- `H_calib` = 建模时刻的升降读数（mm），**一次性实测标定**；标定后 `backHeight == H_calib` 时模型与现状完全一致
- 标定缺口已知（`decisions.md` 记"⏸ 待标定数据"），联调前必须补

### 状态桥（新写 `agx_xarm_bringup/scripts/lift_joint_bridge.py`）

- 订阅 `/lifter_1/LiftMotorStatePub` → 发 `sensor_msgs/JointState`（`lifting_joint`，米）到 `lift_joint/joint_states`
- `realMove_exec.launch:63` 的 `jnt_stat_source` 加第三源 `lift_joint/joint_states`
- `setup_arm.launch` 启动该桥（挂在 `open_lift` 条件下）

### MoveIt 配置

- `joint_limits.yaml`：加 `lifting_joint`（`max_velocity: 0.011`、加速度限值——**必须加**，否则时间参数化按臂速排时间，升降物理跟不上）
- SRDF：新增 `lifting` 组（1 关节）、`xarm7` 组（chain `lift_column_link`→`link6`）；`xarm6` 保持不动；**删除** `:90-91` 两条 disable_collisions
- `kinematics.yaml`：加 `xarm7` 的 KDL 配置（KDL 支持 prismatic）
- `ompl_planning.yaml`：加 `xarm7`、`lifting` 两组规划配置
- `controllers.yaml`：加 `lift_controller`（joints: [lifting_joint]）

### 执行桥（新写 `lift_traj_bridge.py`，仿 `gripper_traj_bridge.py` 模式）

- 起 `lift_controller/follow_joint_trajectory` action server
- 轨迹点（米）→ mm → `/lifter_1/LiftingMotorService`（mode=0 位置控制）
- 轮询 `reachTargetPos` 判完成；preempt → mode=-2 急停

### 静态拷贝

`config/gazebo_ranger_with_xarm6.urdf` 用 xacro 重新生成（它是静态拷贝，不随 xacro 自动更新）

## 2. 安全区校验模块（`agx_xarm_pick` 内独立编译单元）

- 新文件 `include/agx_xarm_pick/safety_zone.hpp` + `src/safety_zone.cpp`，独立类、独立编译，新任务包可直接链接
- 配置 `config/safety_zone.yaml`：`boxes` 列表（name / frame_id / min / max / margin）
- 核心函数：
  - `checkState(RobotState) → 错误码`（入场检查用）
  - `checkTrajectory(RobotTrajectory) → 错误码`（规划后逐 waypoint FK + 全 link AABB 判 box 包含）
- 错误码：`OK / START_OUTSIDE / GOAL_OUTSIDE / TRAJECTORY_OUTSIDE / PLAN_FAILED`

## 3. 接入 `agx_xarm_pick` 原语层

| 位置 | 改动 |
|---|---|
| `MoveToA()` (`agx_xarm_pick_node.cpp:352`) | `plan()` 后插校验；越界 → 重规划（至多 N=3 次）→ 仍失败返回错误码，不执行 |
| `MovePath()` (`:696`) | `computeCartesianPath` 结果校验后再 `execute` |
| `MoveToCatchPose()` (`:1116`) | `move()` 拆成 `plan()`+校验+`execute()` |
| `TaskCallback()` (`:32`) | pick/place 分支入口 enable + 入场检查（区外拒绝，`res.result=false`），出口 disable |
| `demo_pick` | 新增 `launch/demo_pick.launch`：加载 `safety_zone.yaml` + 传 `lift_height`/`retreat_distance`/`use_safety_zone`（它现在无 launch 无 YAML） |

## 4. 新任务包骨架（`src/tasks/` 下）

最小骨架：`package.xml` + `CMakeLists.txt` + 一个示例节点，演示"enable → 规划 → 校验 → 执行 → disable"的用法，链接第 2 节的校验模块。业务逻辑后续填。

## 5. 验证计划（按序）

1. `catkin_make` 全量构建
2. xacro 展开校验（`rosrun xacro xacro ...` 无错）、TF 树出现 `lift_column_link`
3. **HOME 误杀验证**：纯模型下算 HOME 全 link AABB（夹爪前沿距 x=0.331 仅 ~4cm，最容易误杀的点）
4. **demo_pick 回归**：打开 box_link↔link1/link2 自碰撞后全流程跑通（验证 Q16 无误杀）
5. 标定 `H_calib`（实测建模高度对应的升降读数）
6. 升降全行程内 HOME 始终在区内（Q20 推论）
7. 越界用例：故意给一个区外目标 → 应拒绝执行并报 `GOAL_OUTSIDE`

## 6. 风险与注意点

- **时间参数化**：现有 6 关节全部 `has_acceleration_limits: false`，给 `lifting_joint` 加限值后需验证混合轨迹的时间分配合理（联调重点）
- **含升降的轨迹会自动变慢**（11mm/s 主导），这是正确行为不是 bug
- **GUI 滑条直连升降服务**，绕过 MoveIt，一期拦不住（二期运行监控兜底），已确认接受
- 联动规划（`xarm7`）的 IK 收敛速度可能变慢（7 自由度+prismatic），仅新任务显式使用，不影响现有 `xarm6` 流程
- HOME 位姿在升降全行程内必须始终在区内，否则升降指令预检会拒绝合法指令——标定后第一件事实测验证

## 7. 二期（不在本次范围）

- 运行监控：joint_states → FK 校验，越界**原地保持 + 报错**等人工
- planning scene 反向墙：随 enable/disable 增删，**连带拦 RViz/滑条**（已确认接受）
- 标定扩展口（RViz 打点标定区域）
- 已知盲区接受不改：GUI 滑条直连升降服务绕过 MoveIt，墙拦不住，靠运行监控兜底

## 8. 改动文件清单

| 文件 | 动作 |
|---|---|
| `ranger_xarm_description/ranger_xarm6_lifting_2431.xacro` | 改（插入升降关节） |
| `ranger_xarm_lifting_moveit_config/config/ranger_with_xarm6.srdf` | 改（+2 组、−2 碰撞对） |
| `.../config/joint_limits.yaml` / `kinematics.yaml` / `ompl_planning.yaml` / `controllers.yaml` | 改 |
| `.../launch/realMove_exec.launch` | 改（jnt_stat_source） |
| `.../config/gazebo_ranger_with_xarm6.urdf` | 重新生成 |
| `agx_xarm_bringup/scripts/lift_joint_bridge.py` / `lift_traj_bridge.py` | 新写 |
| `agx_xarm_bringup/launch/setup_arm.launch` | 改（启动两桥） |
| `agx_xarm_pick/include/agx_xarm_pick/safety_zone.hpp` + `src/safety_zone.cpp` + `config/safety_zone.yaml` + `launch/demo_pick.launch` | 新写 |
| `agx_xarm_pick/src/agx_xarm_pick_node.cpp` + `include/agx_xarm_pick/agx_xarm_pick.hpp` + `demo/demo_pick.cpp` | 改（接入校验） |
| `src/tasks/<新任务包>/` | 新建骨架 |
| `AGENTS.md` | 更新（"升降不在规划组内"等陈述将过时） |
