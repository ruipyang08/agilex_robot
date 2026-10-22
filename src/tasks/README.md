# src/tasks —— 任务编排层

本目录是机器人**任务编排层**，与 `src/core`（能力/示例）、`src/start`（bringup）平级。

一句话区分三层职责：

| 层 | 目录 | 负责 |
|---|---|---|
| 启动层 | `src/start` | 把硬件跑起来：驱动、bringup、控制台 UI、升降驱动 |
| 能力层 | `src/core` | 机械臂/夹爪/AR 码的底层动作（`agx_xarm_pick`） |
| **任务层** | **`src/tasks`** | **"先干什么、再干什么"的编排与参数** |

## 依赖方向（硬规则）

```
tasks  ──依赖──>  core / start / msg / moveit_config
core, start  ──禁止──>  tasks
```

任务层可以调用能力层暴露的服务、话题、action；**能力层永远不许反向依赖任务**。加新功能时如果发现必须反向依赖，说明抽象错了，要先讨论。

## 目录结构

```
src/tasks/
├── README.md                       # 本文件
├── doc/                            # 跨包文档与实测记录
│   ├── interface.md                # 接口契约（冻结版）
│   ├── decisions.md                # 关键决策与理由
│   ├── measurements.md             # 实测数据（升降标定 / TCP 验证 / 示教读数）
│   └── safety_zone_plan.md         # 设备舱上方安全区 · 一期实施规划（待实施）
├── tests/                          # 离线逻辑测试（不 catkin 编译，直接 python3 跑）
│   ├── run_all.sh                  # 一次跑全部
│   ├── test_frame_gate.py          # 升降坐标系列定门禁（D20）
│   ├── test_motorstop.py           # MotorStop() 限位保护（D20）
│   └── test_bridge.py              # lift_joint_bridge 高度持久化 / 回放（D20）
│   └── test_grasp.py               # 夹取：到位判定 / 急停中断 / 干跑不闭合（D21）
│   └── test_put.py                 # 放入：M6 点位 / 五步顺序 / 黄区豁免与下限 / 干跑（D22）
├── agx_task_msgs/                  # 任务级消息 / 服务 / action
├── agx_task_core/                  # 任务基类 + 能力封装 + 注册表 + Runner
├── agx_task_arm_move_to_pose/      # 【当前任务】机械臂移动到指定位姿
└── agx_task_bringup/               # 汇总：链接所有任务库，产出 runner 可执行 + 总 launch
```

> 除 `doc/`、`tests/` 外的包目录尚未创建，见下方「当前状态」。

## 跑离线测试

```bash
bash src/tasks/tests/run_all.sh
```

**安全约定**：`tests/` 下全部是**纯逻辑测试**——

- **不碰串口、不碰真机**（`C_ROS_Server` 用 `object.__new__` 绕过 `__init__`，不打开串口；`DealAllData` 被替换成计数器）
- **不需要 roscore**（rospy 的 Publisher / Subscriber / get_param / Time / 日志全部打桩）
- **落盘路径强制指向临时目录**，不会污染 `~/.ros/lift_frame_m1.json` 等真实基准文件
- 因此**可以在机器人运行时安全执行**，不会和 `setup_arm.launch` 抢节点名

仍然需要 source ROS 环境（测试要用 `bt_task_msgs` / `sensor_msgs` 消息）。

> 覆盖的缺陷见 `doc/decisions.md` D20：升降柱断电后位置基准丢失 → 坐标系列定门禁 + `MotorStop()` 限位保护修复。
> 改这两处逻辑后请重跑这三个测试。

### 为什么任务包要独立成 `agx_task_*`

- **一个任务一个包**：编译、依赖、参数互不影响，便于按任务交付
- **`agx_task_core` 不含任何具体任务**：只提供基类、能力封装、注册表和 Runner
- **`agx_task_bringup` 是注册清单**：任务通过静态注册宏挂到注册表，需要有个地方把它们链接进同一个可执行文件。加新任务 = 在这里加一行

## 怎么加一个新任务

1. 建包 `agx_task_<任务名>`，依赖 `agx_task_core`、`agx_task_msgs`
2. 写一个类继承 `agx_task::TaskBase`，实现 `name()` / `configure()` / `run()`
3. 用 `AGX_REGISTER_TASK("<任务名>", <类名>)` 注册
4. 参数默认值写进本包 `config/<任务名>.yaml`
5. 在 `agx_task_bringup` 的 `CMakeLists.txt` 里加一行链接本包
6. 写本包 `launch/<任务名>.launch`（内部 include 需要的 bringup，例如 `setup_arm.launch`）

接口细节见 `doc/interface.md`。

## 文档索引

| 文档 | 内容 |
|---|---|
| `doc/interface.md` | 接口契约：消息定义、`TaskBase`、`TaskContext`、四个 capability、Runner 行为、跨接口约定 |
| `doc/decisions.md` | 关键决策记录：为什么这么设计、放弃了哪些方案 |
| `doc/measurements.md` | 实测数据记录：升降柱标定、TCP 验证、示教读数、放入点 M6、`grip_state` 语义 |
| `doc/safety_zone_plan.md` | 设备舱上方安全区：已确认的设计决策、一期实施规划、验证计划（待实施） |

**放置规则（重要）**

- **实际生效的参数**放各任务包的 `config/*.yaml`（代码要读）
- **实测过程与来源**放 `doc/measurements.md`（人要追溯）
- **同一个数只写一遍**：文档写"怎么测的、原始读数、结论"，YAML 写"实际生效的值"

## 当前状态

| 项 | 状态 |
|---|---|
| 接口契约 | ✅ 已对齐，见 `doc/interface.md` |
| 关键决策 | ✅ 已记录，见 `doc/decisions.md` |
| 升降柱标定 | ✅ 已完成 2026-09-28（M1：方向/线性/零点 875mm/行程 600mm） |
| TCP（指尖下沿） | ✅ 已实测并写入 URDF：闭合 **0.170 m**（2026-09-29 复测确认，取代初测 0.230 m） |
| ✅ 夹爪模型"短 0.083 m"**推断作废**（2026-09-29 复测：只短 23 mm） | 实测 0.170 vs 模型 0.147，差 23 mm 且方向保守；`decisions.md` D16 可更新 |
| 目标平面示教 | ⏸ 待上机器示教 |
| 夹取（移到红点 + 闭合） | ✅ 服务 + 控制台按钮已实现，离线测试全过（D21）；⏸ 真机端到端未验证 |
| 放入（M6 六个示教点，`put <1~6>`） | ✅ 服务 + CLI + 六红球 Marker 已实现，离线测试与假臂 headless 验收全过（D22）；⏸ 真机端到端未验证 |
| `grip_state` 语义（夹住/空夹） | ⏸ 仍未实测（M4）。夹取**不依赖**它，用"位置停止变化"判到位 |
| `agx_task_*` 包实现 | ⏸ 未开始 |

## 相关改动（不在本目录，但属于同一批工作）

| 位置 | 改动 | 状态 |
|---|---|---|
| `moveit_config/ranger_xarm/ranger_xarm_description/ranger_xarm6_lifting_2431.xacro` | 把固定关节 `ranger_with_xarm` 替换为 prismatic `lift_joint`（`box_link` → 可动 → `arm_base`），让 MoveIt 知道升降高度 | ✅ 已完成（M1 标定 2026-09-28；原表中写的 `ranger_xarm_2431.xacro` 路径有误，实际加载的是本文件） |
| `moveit_config/ranger_xarm/ranger_xarm_description/ranger_xarm6_lifting_2431.xacro` | 增加固定的 `tcp_link`（指尖下沿，实测 **0.170 m**，2026-09-29 复测修正） | ✅ 已完成 |
| `moveit_config/ranger_xarm/ranger_xarm_description/ranger_xarm6_lifting_2431.xacro` | ✅ 夹爪模型"短 83mm"推断作废（2026-09-29 复测，实际只短 23 mm）；TCP 改为 0.170 m | ✅ 见 `measurements.md` M2 |
| `src/start/agx_xarm_bringup/scripts/lift_joint_bridge.py` | 把 `/lifter_1/LiftMotorStatePub` 的 `backHeight` 转成 `lift_joint` 关节状态（**只读**，不驱动升降） | ✅ 已完成 |
| `src/start/agx_xarm_bringup/launch/setup_arm.launch` | 启动上面这个桥接节点 | ✅ 已完成 |
| `src/start/agx_xarm_bringup/CMakeLists.txt` | `catkin_install_python` 注册 `lift_joint_bridge.py` | ✅ 已完成 |
| `src/moveit_config/ranger_xarm_lifting_moveit_config/launch/realMove_exec.launch` | `jnt_stat_source` 加上升降关节来源 `lift_slider/joint_states` | ✅ 已完成 |
| `src/start/agx_xarm_bringup/scripts/lift_zone_viz.py` | 常驻 RViz Marker：绿=升降柱碰撞包围盒（212×212×650，随 `lift_joint` 移动）、红=臂底座轮廓 | ✅ 已完成 2026-09-28 |
| `src/start/agx_xarm_bringup/launch/setup_arm.launch` | 启动 `lift_zone_viz` | ✅ 已完成 |
| `src/start/agx_xarm_bringup/rviz/arm_display.rviz` | 增加 `Marker` 显示（`/lift_zone/marker`，默认可见） | ✅ 已完成 |
| `src/start/agx_xarm_bringup/CMakeLists.txt` + `package.xml` | 注册 `lift_zone_viz.py`；补 `visualization_msgs`/`geometry_msgs` 依赖 | ✅ 已完成 |
| `src/tasks/agx_plane_target/srv/PutPoint.srv` | 放入服务定义（`int32 point` 1~6 -> `success`/`message`），本包第一个消息定义（D22） | ✅ 已完成 2026-10-14 |
| `src/tasks/agx_plane_target/scripts/plane_target_marker.py` | 放入服务（上方点 -> 直线下降 -> 松开 -> 直线抬高 -> 回 initial）、`put_status`、六红球 MarkerArray、黄区约束的 `goal_point`/`floor` 扩展 | ✅ 已完成 2026-10-14 |
| `src/tasks/agx_plane_target/scripts/put` | 终端命令 `put <1~6>` / `putN` / `put status` / `put stop` / `put install`（自动补 ROS 环境） | ✅ 已完成 2026-10-14 |
| `src/start/agx_xarm_bringup/rviz/arm_display.rviz` | 新增 `MarkerArray` 显示项 `/plane_target/put_points`（ns `put_points`） | ✅ 已完成 2026-10-14 |
| 升降柱碰撞区并入 MoveIt 碰撞模型（`lift_inner_link` visual+collision + SRDF 3 条豁免） | 干跑已验证可行（见 `measurements.md` M5），**尚未落地**，等复测/确认 | ⏸ |
