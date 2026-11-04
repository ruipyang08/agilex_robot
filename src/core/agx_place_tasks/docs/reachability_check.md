# 机载架可达性试规划（交接文档第 7 节的可执行版）

目的：在写任何放置轨迹之前，先回答一个问题——**这台机器人的臂能不能够到自己的架子**。
够不到就要改机械方案（比如把架子挪到底盘尾部），这件事必须先行。

工具：`scripts/check_rack_reach.py` + `print_rack_slots`（C++）。

**只规划，不执行**：脚本里没有 `go()`、没有 `execute()`、没有任何轨迹 action 客户端。
第一次真机低速单步仍然必须由人手动做。

## 1. 先用实测 profile 填好架子

`config/onboard_rack_profiles.yaml` 的字段含义见 `configuration.md`。
未标定（`qualified: false`）也能跑，但脚本会打印警告：此时的结论只证明"工具能用"，
不证明"机器人放得下"。

## 2. 有真机（推荐）

```bash
roslaunch agx_xarm_bringup setup_arm.launch plane_target_execute:=false   # 只起模型和 MoveIt
rosrun agx_place_tasks check_rack_reach.py \
    --rack-file $(rospack find agx_place_tasks)/config/onboard_rack_profiles.yaml \
    --rack-id onboard_rack_v1 \
    --roll-sweep 45 --planning-time 10 --json-out /tmp/reach.json
```

`setup_arm.launch` 会发布 `link_base_lift0`、`deck_top` 两个静态 TF，profile 的
`parent_frame` 就是它们。

## 3. 没有真机（假执行器）

```bash
roslaunch ranger_xarm_lifting_moveit_config demo.launch use_rviz:=false
rosrun agx_place_tasks check_rack_reach.py --rack-file <rack.yaml> --rack-id <id> \
    --reference-frame link_base_lift0 --roll-sweep 45
```

注意：demo 里夹爪和部分轮子缺 mesh 时会退化成"无碰撞几何"，因此**不覆盖挤压/自碰撞判断**，
只用于验证脚本逻辑和粗略可达性。

## 4. 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--rack-file` / `--rack-id` | 必填 | 走 `print_rack_slots`，与 C++ 同一套校验 |
| `--slots` | 全部 | 只测部分槽位，如 `1,3,6` |
| `--group` | `xarm6` | MoveIt 规划组 |
| `--tip-link` | 组的末端 | 一般不用改 |
| `--reference-frame` | profile 的 `parent_frame` | 姿态参考系 |
| `--start-state` | `initial` | 从 SRDF 命名位姿开始规划；空字符串表示从当前状态 |
| `--planning-time` | `5.0` | 每段规划的时间上限，秒 |
| `--roll-deg` | `0` | 绕工具轴（插入方向）的偏航 |
| `--roll-sweep` | 关 | 例如 `45`：每个槽位试 0/45/…/315，报出哪些偏航可行 |
| `--json-out` | 无 | 输出完整报告（每段轨迹的关节终点、裕量、joint5 最小值） |

每个槽位分两段：`approach`（槽位上方的接近点）→ `insert`（槽中心），
第二段从第一段的终点继续规划，和真实动作顺序一致。

## 5. 怎么读结果

| 输出 | 含义 |
|---|---|
| `OK` + `margin` | 规划成功；`margin` 是终点各关节到行程限位的最小距离（rad），越小越危险 |
| `note: passes within 0.15 rad of the wrist singularity` | 路径贴着腕部奇异（joint5≈0），实机低速也可能抖动或不可控 |
| `insert:TIMED_OUT` | 在给定时间内没找到无碰撞解。**不等于物理不可达**，先加大 `--planning-time` |
| `insert:GOAL_IN_COLLISION` | 目标位姿本身碰撞（架子/相机/升降柱），这是硬伤 |
| `insert:NO_IK_SOLUTION` | 该偏航下 IK 无解，换 `--roll-sweep` |
| `working yaw values: ...` | 可用偏航集合；应把它写成真实的放置姿态约定 |

判定顺序建议：

1. 先 `--planning-time 10` 排除"只是没算出来"；
2. 看 `GOAL_IN_COLLISION` / `START_STATE_IN_COLLISION`——这两个是真问题；
3. 看 `margin` 和 joint5 最小值——轨迹虽然成功但贴限位/贴奇异，实机不允许；
4. 全部槽位都只有奇异解或都失败 → 回到交接文档第 7 节的结论：架子位置要改。

## 6. 当前状态

- 脚本已在隔离 catkin 工作区构建，并在假执行器 `move_group` 上跑通（连接、规划、度量、JSON、退出码）。
- 2026-11-04 用假执行器 + 本仓库 URDF 跑过一次（`tcp_link` 作末端，教出的姿态，planning-time 10 s）：
  **6/6 槽位全部可规划，无碰撞**；进给段关节限位裕量 0.36-0.64 rad，两段各 2-3 s。
  **但六个插入位形的 |joint5| 只有 0.01-0.13 rad**，贴着腕部奇异（8 个偏航都一样，是“竖直工具 + 这个位置”的固有结果）。
  真机执行时下降必须走笛卡尔直线并低速，不能靠关节空间插值。
- 2026-11-04 真机运行（生产栈 setup_arm.launch + 真实 xArm 驱动，plane_target_execute:=false，未执行任何运动）：
  **6/6 槽位可规划，无碰撞**；insert 段 final |joint5| = 0.011-0.126 rad，与模型结果逐位一致。
  分别从 SRDF initial 和从真实当前关节状态各跑一遍，末态关节几乎相同，
  说明“贴奇异”是这六个位姿本身的几何属性，不是起始种子或仿真造成的。
  真机当前 tcp 实测 (0.278, 0.010, 0.229) / RPY(-180.0, 0.5, 2.6) 度，
  与 M6 记录的姿态一致，坐标系对齐无误。
  末态关节形如 q=[0.024, 0.314, -0.424, 0, 0.110, 0]：joint4 与 joint6 全程约等于 0，
  腕部只在机械臂平面内弯曲，下降过程中没有换分支。
  规划成功不等于实机运动合格：夹爪碰撞体、负载与速度环仍要在低速单步里确认。
- 宿主 `run_robot_workflow` 的放置动作适配器仍未实现，正是等这一步的结论。
