# xArm6 + 升降柱：正/逆运动学解算过程与 RViz 拖动跟随设计

> **状态**：设计稿（**未实施**，未改动任何代码/URDF/launch）
> **日期**：2026-09-28
> **位置**：`src/tasks/doc/kinematics_ik_plan.md`（与 `safety_zone_plan.md` 同风格）
> **拟建交付物**：`src/core/agx_xarm_kinematics`（FK 库 + 2 个脚本）＋ `arm_display.rviz` 增加 InteractiveMarkers 显示
> **参考**：`ranger_xarm6_lifting_2431.xacro`、`xarm6.urdf.xacro`、`measurements.md`(M1/M2)、`decisions.md`(D17–D19)、`joint_slider_panel.cpp`、`xarm_hw.cpp`

---

## 0. 摘要（三句话版）

1. **正运动学**：把 `base_link → tcp_link` 写成 11 段齐次变换连乘，其中第 2 段是升降柱棱柱关节
   `lift_joint`（0~0.6 m，+z），广义坐标 `x = [d, q1..q6]`（7 维）；同时给解析雅可比。
2. **逆解**：每个控制拍用**带边界约束的优化**（`scipy.optimize.least_squares`, trf）最小化
   `||p(x) − p_des||`（可选姿态项 + 正则项），**位置/速度/加速度限制做成逐关节可行域盒子**
   （硬约束，不是罚项），用上一拍解热启动。
3. **RViz 拖动跟随**：6-DoF 交互 marker 作为拖动目标，30 Hz 闭环"读关节反馈 → 解一步 → 限幅 →
   发单点 JointTrajectory"；marker 的视觉位置在收敛后吸附在 `tcp_link`（两指指尖中点），
   真机下发时**锁死升降**（升降只参与仿真解算）。

---

## 1. 正运动学（FK）

### 1.1 记号与约定

- 齐次变换 `T = [[R, t], [0, 1]]`；`Trans(a,b,c)` 平移；`Rz(q)`/`Rx(q)` 绕轴旋转；
  `RPY(r,p,y) = Rz(y)·Ry(p)·Rx(r)`（URDF 固定轴 roll-pitch-yaw 约定）。
- 广义坐标 `x = [d, q1, q2, q3, q4, q5, q6]^T`，**升降排第 0 位**；
  `d` 单位 m（0 = 机械最低），`q_i` 单位 rad。
- 根坐标 `base_link`（底盘上表面中心，离地约 0.32 m，+x 车头、+y 左、+z 上）；
  末端 `tcp_link`（夹爪闭合时两指尖下沿的中点）。
- 除标注外，所有位姿都表达在 `base_link` 系。

### 1.2 运动链逐段参数（对照 URDF，按顺序）

| # | 父 → 子 | 类型 | origin xyz (m) | origin rpy (rad) | axis | 关节变量 |
|---|---|---|---|---|---|---|
| 0 | `base_link → box_link` | fixed | `-0.02 0 0` | `0 0 0` | — | — |
| 1 | `box_link → arm_base` | **prismatic** | `-0.17 0 0.555` | `0 0 0` | `0 0 1` | `d ∈ [0, 0.6]` |
| 2 | `arm_base → link_base` | fixed | `0 0 0` | `0 0 0` | — | — |
| 3 | `link_base → link1` | revolute | `0 0 0.267` | `0 0 0` | `0 0 1` | `q1` |
| 4 | `link1 → link2` | revolute | `0 0 0` | `-1.5708 0 0` | `0 0 1` | `q2` |
| 5 | `link2 → link3` | revolute | `0.0535 -0.2845 0` | `0 0 0` | `0 0 1` | `q3` |
| 6 | `link3 → link4` | revolute | `0.0775 0.3425 0` | `-1.5708 0 0` | `0 0 1` | `q4` |
| 7 | `link4 → link5` | revolute | `0 0 0` | `1.5708 0 0` | `0 0 1` | `q5` |
| 8 | `link5 → link6` | revolute | `0.076 0.097 0` | `-1.5708 0 0` | `0 0 1` | `q6` |
| 9 | `link6 → dh_robotics_ag95_base_link` | fixed | `0 0 0` | `0 0 0` | — | — |
| 10 | `dh_robotics_ag95_base_link → tcp_link` | fixed | `0 0 0.170` | `0 0 0` | — | — |

出处：第 0/1/2/9/10 段在 `ranger_xarm6_lifting_2431.xacro`；第 3–8 段在 `xarm6.urdf.xacro`
（`xarm6_robot_macro.xacro` 的 `world_joint` 默认 `xyz="0 0 0"`，故第 2 段是恒等）。

**注意**：URDF 里直角写的是 `±1.5708` 而不是 `±π/2`（差约 4e-6 rad）。FK **原样照抄**，
这样 `--check-urdf` 能做到逐位 bit 级一致；不要"顺手改成 pi/2"。

### 1.3 FK 递推公式

```
T ← I
for 第 i 段 (O_i = Trans(xyz_i) · RPY(rpy_i)):      # O_i 只含 origin，不含关节变量
    A_i ← T · O_i                                   # 关节坐标系（axis 在此系中表达）
    if revolute:   T ← A_i · Rz(q_i)
    if prismatic:  T ← A_i · Trans(0, 0, d)
    (记录 A_i，供雅可比使用)
p_tcp(q) = T[:3, 3]        R_tcp(q) = T[:3, :3]
```

伪代码约 15 行，纯 numpy 矩阵连乘，单次 FK < 10 µs（7 个 4×4 乘法）。

### 1.4 "夹爪正中间"的定义

- 采用现有 `tcp_link`：`link6 →（恒等）→ dh_robotics_ag95_base_link → (0,0,+0.170 m)`。
  URDF 注释原文："夹爪闭合状态下两指尖下沿的中点"，正是"夹爪正中间"。
- 0.170 m 是 M2 **2026-09-29 复测确认值**（初测 0.230 m 作废；"模型短 83 mm"的旧推断同样作废，实差 23 mm）。
  FK 不硬编码这个印象：`--check-urdf` 每次都拿 URDF 现值比对，URDF 改了 FK 会报差异。
- 可选扩展（非本期）：若要让"中心"随 `finger_joint` 开合变化，可把 4 个 mimic 关节
  （`left/right_proximal_phalanx`、`distal_phalanx`、`rod`，倍率在 URDF 里）也建进模型，
  取两指尖中心。当前不做。

### 1.5 解析雅可比

对每个关节 i，用 1.3 记录的 `A_i`（旋转前）：

```
z_i = A_i[:3, :3] · [0, 0, 1]^T        # 关节轴在 base_link 系
p_i = A_i[:3, 3]                       # 关节原点在 base_link 系
revolute:  J_i = [ z_i × (p_tcp − p_i) ;  z_i ]   (6×1)
prismatic: J_i = [ z_i ; 0 ]                      (6×1)
J = [J_0 J_1 ... J_6]                  (6×7)
```

用途：位置残差的解析梯度（见 2.2），以及离线验证 `J` 的数值差分（每列一次 FK）。

### 1.6 升降柱（第 2 段）

- `d` 实机来源：`/lifter_1/LiftMotorStatePub` 的 `backHeight`（mm，1:1）/1000；
  `initState == false` 时该值不可信，节点按"无反馈"处理（保持上一拍，绝不当作 0）。
- URDF 限位：`0 ~ 0.6 m`，axis `+z`（抬升为 +），实测速度 `0.011 m/s`（M1）。
- FK 效果：`link_base` 随 `d` 平移，`T_base_tcp` 自动含高度，无需其他特殊处理。
- `link_base_lift0` / `deck_top` 是上一批工作加的**静态参考系**，只用于别的任务/可视化，
  **不要**混进 FK 链（FK 里只有运动的 `link_base`）。

### 1.7 自检（验收 FK 建模正确）

| 命令 | 做什么 | 通过标准 |
|---|---|---|
| `xarm_fk.py --check-urdf` | 子进程跑 `xacro` 展开 2431 模型 → `urdf_parser_py` 解析 → BFS 出 `base_link→tcp_link` 链 → 与内置表逐段比对 (type, xyz, rpy, axis) | 逐段完全一致，退出码 0 |
| `xarm_fk.py --demo` | 几组 `x` 打印 `p_tcp/R_tcp` | 人工抽查 |
| `xarm_fk.py --check-jac` | 每列与数值差分比 | 最大差 < 1e-6 |
| `xarm_fk.py --check-tf`（真机在跑，可选） | 与 `tf_echo base_link tcp_link` 对拍 | 位置 < 1e-4 m |

`--check-urdf` 需要 `source` 过工作空间（`rospack`/`xacro`），不需要 ROS master。

---

## 2. 逆解：逐 step 约束的优化

### 2.1 优化问题

第 k 拍（周期 `dt = 1/rate_hz`）在上一拍解附近解：

```
min over x:   w_p² · ||p(x) − p_des||²          # 位置误差（需求原文）
            + w_o² · ||φ(x)||²                  # 姿态误差，"保持启动姿态"时 w_o > 0
            + w_r² · ||x − x_ref||²             # 正则，x_ref = 实测 q_fb，抑制抖动/漂移
```

以残差向量形式交给 `least_squares`：

```
r(x) = [ w_p·(p(x) − p_des) ;  w_o·φ(x) ;  w_r·(x − x_ref) ]
```

- `w_o = 0` 即退化为"只跟位置"（严格符合 `||p−p_des||min`）。
- 默认采用"位置 + 保持启动姿态"：启动瞬间把 `R_des ← R_tcp(x_启动)` 固定为常数。

### 2.2 姿态残差与解析梯度

```
R_err(x) = R_des^T · R(x)          # SO(3)
φ(x)     = Log(R_err)              # 旋转向量（轴角），∈ R³
```

梯度（精确式）：

```
∂φ/∂x_i = J_r^{-1}(φ) · R(x)^T · ω_i
```

其中 `ω_i` 是雅可比 `J` 的角速度行（第 3~5 行）第 i 列，`J_r^{-1}` 是右雅可比之逆：

```
J_r^{-1}(φ) = I − (1−cosθ)/θ² · [φ]× + (θ−sinθ)/θ³ · [φ]×²,   θ=|φ|
θ→0 时取极限 I − ½[φ]×（实现里 θ < 1e-6 直接走这个分支）
```

位置行用 1.5 的 `J_v` 解析；正则行是 `w_r·I`。**所有行都可解析**；
代码里保留 `--fd-jac` 开关用有限差分对拍（每拍多 ~7 次 FK，也完全够快）。

### 2.3 每拍硬约束（位置/速度/加速度）——本方案的核心

记：`x_{k−1}` 上一拍解、`x_{k−2}` 上上拍解、`v_{k−1} = (x_{k−1} − x_{k−2})/dt`、
`q_fb` 本拍实测、`vmax/amax` 逐关节参数（见 2.8）。

| 约束 | 数学形式 | 物理含义 |
|---|---|---|
| 位置 | `qmin ≤ x ≤ qmax` | 关节限位（URDF/固件） |
| 速度 | `|x − x_{k−1}| ≤ vmax·dt` | 单拍位移上限 = 速度限制 |
| 加速度 | `|(x − x_{k−1})/dt − v_{k−1}| ≤ amax·dt` ⇔ `|x − (x_{k−1} + v_{k−1}·dt)| ≤ amax·dt²` | 单拍速度变化上限 |
| 防跑偏 | `|x − q_fb| ≤ vmax·dt` | 指令不领先实测超过一拍，防滞后累积/开环放大 |

**逐关节求交 → 可行域盒子（硬约束）：**

```
lb = max( qmin,  x_{k−1} − vmax·dt,  x_{k−1} + v_{k−1}·dt − amax·dt²,  q_fb − vmax·dt )
ub = min( qmax,  x_{k−1} + vmax·dt,  x_{k−1} + v_{k−1}·dt + amax·dt²,  q_fb + vmax·dt )
```

三点说明：

1. 约束是**可行域边界**而不是罚项：`trf` 严格保界，解不可能越限（只有 1e-9 量级数值余量，
   见 2.8 的 `limit_margin`）；
2. 盒子是逐关节的，所以两个求解器都能直接用；
3. **边界冲突**（`lb > ub`：实测滞后过大或反馈噪声）：该拍退化为
   `x = clip(q_fb, qmin, qmax)`（原地保持），标记 `hold`，日志提示，绝不放大动作。
   仿真中不会出现（`q_fb` 即上一拍解附近）；真机偶发属正常。

### 2.4 求解器与热启动

| 模式 | 调用 | 说明 |
|---|---|---|
| `lsq`（默认） | `least_squares(r, x0, jac=r_jac, bounds=(lb,ub), method='trf', xtol=ftol=gtol=1e-8, max_nfev=50)` | 盒子直接作为 bounds；速度快、稳 |
| `slsqp`（对照） | `minimize(f=½‖r‖², x0, jac, method='SLSQP', bounds=(qmin,qmax), constraints=[速度, 加速度, 防跑偏的显式不等式])` | 答辩时展示"显式约束"的标准形式；`|·|` 用两侧写法拆成 2 条不等式，避免不可导 |

- 初值 `x0 = clip(x_{k−1}, lb, ub)`（**热启动**，通常 3~8 次迭代收敛）。
- 超时保护：单拍解算 > `dt/2` 或求解器失败 → 本拍**不下发新值**（保持上一拍命令），
  记录失败计数；连续 5 拍失败发 WARN 并停发（进入 hold）。

### 2.5 状态机与初始化

```
启动：  x_{k−1} = x_{k−2} = q_fb(实测)；v_{k−1} = 0     # 首拍天然满足速度/加速度限制
每拍：  target ← (marker 拖动点，见第 3 节)
        (lb,ub) ← 2.3 的盒子
        x_sol   ← 求解（失败则 hold）
        记录/统计 → 志愿者下发（见 3.4）→ x_{k−2}←x_{k−1}; x_{k−1}←x_sol
```

### 2.6 限幅验收（"怎么证明每步都满足"）

运行结束后用**与解算完全相同的差分公式**从记录序列反算，输出合规报告：

```
v_k = (x_k − x_{k−1})/dt                →  max_j,k |v_k,j| / vmax_j  ≤ 1 + 1e-6
a_k = (v_k − v_{k−1})/dt                →  max_j,k |a_k,j| / amax_j  ≤ 1 + 1e-6
位置：qmin − 1e-9 ≤ x_k ≤ qmax + 1e-9    →  0 越限
末端：||p(x_k) − p_des,k||               →  收敛曲线，静止目标 < 1 mm
耗时：单拍解算时间                        →  max < dt/2
```

输出：控制台报告 + `results/ik_limits_*.csv` + matplotlib 三联图
（误差/速度/加速度，各自叠加限制线）。离线脚本还支持：随机可达目标蒙特卡洛
（从随机 `x` 正解出 `p_des` 再扰动 1~2 cm 反解，统计成功率）。

### 2.7 升降自由度：锁定 vs 自由（重要）

| 模式 | 行为 | 用在哪 |
|---|---|---|
| `lift_mode = free` | 7 维一起解；升降受 `vmax=0.011 m/s` 限制，可"帮忙"够高/远的目标；输出里有升降轨迹 | **仅仿真**：离线 demo、`enable_arm=false` 的纯 RViz 演示 |
| `lift_mode = locked`（实机必须） | `lb[0] = ub[0] = d_fb`，等效 6 维 | 真机下发 |

原因：升降柱不在 `xarm6_traj_controller` 里、实机不闭环。若真机模式下按 7 维解，
机械臂 6 轴执行的是"升降会动"假设下的解 → 实际 TCP 差一个升降位移的误差。
节点在 `enable_arm=true && lift_mode=free` 时**拒绝启动**并给出明确报错。

### 2.8 参数表（`config/ik.yaml` 草案，实施时以此为唯一生效值）

```yaml
ik:
  rate_hz: 30                 # 控制周期 dt = 1/30 s
  solver: lsq                 # lsq | slsqp
  max_nfev: 50
  position_weight: 1.0        # w_p
  orientation_weight: 0.5     # w_o；0 = 只跟位置；>0 = 保持启动姿态
  regularization_weight: 1.0e-3   # w_r
  lift_mode: locked           # locked | free
  limit_margin: 1.0e-9        # 盒子内缩，防数值越界误判

joints:                       # 顺序 = [d, q1..q6]
  lift_joint: {min: 0.0,     max: 0.6,     vel: 0.011,  acc: 0.01}   # URDF + M1 实测
  joint1:     {min: -6.2832, max: 6.2832,  vel: 0.4,    acc: 2.0}    # URDF ±2π
  joint2:     {min: -2.059,  max: 2.0944,  vel: 0.4,    acc: 2.0}    # URDF
  joint3:     {min: -3.927,  max: 0.19198, vel: 0.4,    acc: 2.0}    # URDF（注意上限很小）
  joint4:     {min: -6.2832, max: 6.2832,  vel: 0.4,    acc: 2.0}    # URDF ±2π
  joint5:     {min: -1.69297,max: 3.14159, vel: 0.4,    acc: 2.0}    # URDF
  joint6:     {min: -6.2832, max: 6.2832,  vel: 0.4,    acc: 2.0}    # URDF ±2π
# 说明：URDF 标称速度 3.14 rad/s、加速度无标称。0.4/2.0 是"演示可接受、真机稳妥"的
#       默认值，比标称低一个量级；要更快只改这里。

target_box:                   # 目标点裁剪（base_link 系，可选），防拖到不可达/下方
  enabled: true
  min: [-0.05, -0.45, 0.60]   # 暂定值：z 下沿取安全区下方，x/y 取可达包络
  max: [ 0.80,  0.45, 1.70]
```

---

## 3. RViz 拖动跟随（`marker_ik_teleop.py`）

### 3.1 结构与话题

| 名称 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/xarm/joint_states` | `sensor_msgs/JointState` | 订阅 | 6 臂关节反馈（joint_state_controller, 50 Hz） |
| `/lifter_1/LiftMotorStatePub` | `bt_task_msgs/LiftMotorMsg` | 订阅 | `backHeight`(mm) + `initState`（50 Hz） |
| `/ik_marker/update` | `visualization_msgs/InteractiveMarkerUpdate` | InteractiveMarkerServer 发布 | RViz 加 `InteractiveMarkers` 显示即可拖动 |
| `/ik_marker/debug` | `visualization_msgs/Marker` | 发布 | 目标点（红球）+ FK 实际（绿球）+ 误差文本（可选） |
| `/xarm/xarm6_traj_controller/command` | `trajectory_msgs/JointTrajectory` | 发布 | **仅 `enable_arm=true` 时**，单点轨迹 |
| `~stop` | `std_srvs/Trigger` | 服务 | 停止下发（hold），marker 定在当前 FK |

marker 定义：frame `base_link`，名字 `tcp_target`，6-DoF（球体 + 三轴箭头 `MOVE_AXIS`；
姿态分量默认不给控制柄，因为默认只保持启动姿态；想要"姿态也跟"时再加 `ROTATE_3D`
并把 `w_o>0`、`R_des` 改为跟随 marker 旋转）。

### 3.2 每拍流程（30 Hz 定时器，伪代码）

```python
on_timer():
    if now - last_arm_state  > 0.5 s:  warn; return          # 反馈超时 → 不动
    if now - last_lift_state > 0.5 s:  warn; return          # 升降未知 → 不动
    q_fb, d_fb = 最新反馈
    target = clamp(target, target_box)                        # 3.5
    x_sol, info = ik_stepper.step(target, q_fb, d_fb)         # 第 2 节
    if info.failed:  hold; return
    if enable_arm and info.converged_or_improving:
        publish_single_point_traj(x_sol)                      # 3.4，变化 < 1e-4 rad 时跳过
    update_debug_markers(target, fk(q_fb))
    update_marker_pose_state_machine()                        # 3.3
```

### 3.3 marker 语义（"保持在夹爪正中间"的实现）

三个状态，保证**松手不跳变、收敛后一定回到夹爪中心**：

| 状态 | 进入条件 | marker 位姿 | target |
|---|---|---|---|
| `DRAGGING` | 收到 `MOUSE_DOWN`，或最近 1 s 内有 `POSE_UPDATE` | 由 RViz 客户端控制，**节点不 setPose**（避免抢鼠标/抖动） | = 反馈位姿 |
| `SETTLING` | 松手（`MOUSE_UP`/超时）且 `‖p(q_fb) − target‖ > 2 mm` | 每拍 `setPose(target)`（仍显示目标点） | 保持不变，臂继续收敛 |
| `LOCKED` | `‖p(q_fb) − target‖ ≤ 2 mm` 且非拖动 | `target ← FK(tcp)(q_fb)`；`setPose(target)` | = 实测 FK(tcp) |

- `LOCKED` 下 marker 就贴在夹爪两指中间，臂处于"跟着自己"的闭环，稳态零漂移；
- 升降被人为移动时：`q_fb` 不变、`FK(tcp)` 抬高 → `target` 同步重锚 → 臂不需要动，
  marker 跟着夹爪一起升降（符合直觉）；
- 参数 `~snap_on_release:=true` 可改为"松手立即重锚"（deadman 手感：松手即停），默认 false。

### 3.4 真机下发通道

- 单点 `JointTrajectory`：`joint_names=[joint1..joint6]`，
  `positions = x_sol[1:7]`（**不带升降**），`time_from_start = 1/rate_hz`（≥ 0.02 s）。
- 节流：所有关节变化 < `1e-4 rad` 时跳过；但最长每 0.5 s 强制发一次（保持控制器目标新鲜）。
- 与底层的关系（已核对）：`xarm_hw` 100 Hz 插值，指令变化即 `setServoJ`；
  硬件接口对 **>0.25 rad 的跳变直接拒绝**——我们的限幅保证每拍 ≪ 0.25 rad
  （vmax·dt = 0.4/30 ≈ 0.013 rad），天然满足。
- **互斥**：teleop 运行期间不要用 MoveIt 运动面板 / 关节滑条（同一个控制器会互相抢）；
  节点启动时可在日志里提示。

### 3.5 目标裁剪盒（暂定）

- 目的：拖动时防止目标跑到地板下、柱后或明显不可达处（**不是碰撞检测**，只是省心的夹紧）。
- 默认值见 2.8 `target_box`；与 `safety_plane_viz.py` 的暂定安全区数值同源（实施时对齐、
  数值只写一处），`enabled: false` 可关。
- 拖到盒子外时：目标被夹在盒面，`SETTLING` 会一直有残差 → 日志/调试 marker 上标"未达"（红球）。

### 3.6 异常与安全

| 项 | 行为 |
|---|---|
| 默认姿态 | `enable_arm: false`（dry-run，只算不下发），必须显式 `_enable_arm:=true` |
| 反馈超时 | 保持上一拍，WARN 限流 1 Hz |
| 求解失败 | 保持上一拍，不跳变；连续 5 拍转 hold |
| 关节跳变 | 下发前检查 `max|Δx| ≤ vmax·dt`（构造上必然成立，纯防御性断言） |
| 急停 | 物理急停随时有效；`~stop` 服务把节点切 hold；Ctrl-C 停止进程（控制器保持最后目标，机械臂停在原地） |
| 启动 | 不等同于"使能"：`setup_arm.launch` 的拖动示教/滑条功能与 teleop 不共时使用 |
| 真机前提 | 周围清场、手边急停、先 `enable_arm:=false` 干跑观察 |

---

## 4. 文件清单

```
新增  src/core/agx_xarm_kinematics/
      ├── package.xml                  # rospy sensor_msgs trajectory_msgs visualization_msgs
      │                                # interactive_markers geometry_msgs std_srvs bt_task_msgs
      ├── CMakeLists.txt               # catkin_install_python 注册 3 个脚本，安装 launch/config
      ├── config/ik.yaml               # 2.8 的参数表（唯一生效值）
      ├── launch/marker_ik_teleop.launch
      └── scripts/
          ├── xarm_fk.py               # 第 1 节：FK 库 + 自检 CLI（需求 1）
          ├── ik_limits_demo.py        # 第 2 节：离线路径 + 限幅验收 + CSV/图（需求 2）
          └── marker_ik_teleop.py      # 第 3 节：RViz 拖动跟随（需求 3）

修改  src/start/agx_xarm_bringup/rviz/arm_display.rviz
      └── 增加一个 rviz/InteractiveMarkers 显示（Update Topic: /ik_marker/update）

不改  URDF / SRDF / launch / 现有脚本（lift_joint、tcp_link 已就绪，无需动）
```

共享方式：`xarm_fk.py` 既是库也是脚本；另两个脚本放同目录，`import xarm_fk` 直接可用
（rosrun/roslaunch 执行时脚本目录在 `sys.path[0]`）。

---

## 5. 运行与验证步骤（离线 → 干跑 → 真机）

```bash
source /opt/ros/noetic/setup.bash
source ~/agilex_ws/devel/setup.bash

# 构建（新包第一次建议全量）
cd ~/agilex_ws && catkin_make && source devel/setup.bash

# A. FK 自检（不需要 ROS master；需要 source 环境）
rosrun agx_xarm_kinematics xarm_fk.py --check-urdf --check-jac --demo

# B. 逆解限幅离线验证（不需要机器人）
rosrun agx_xarm_kinematics ik_limits_demo.py --path circle --solver lsq
rosrun agx_xarm_kinematics ik_limits_demo.py --path waypoints --solver slsqp --plot
#    报告 0 越限，图/CSV 在 results/

# C. RViz 干跑（marker 贴夹爪中心、日志误差收敛，机械臂不动）
roslaunch agx_xarm_bringup setup_arm.launch          # 真机 bringup + MoveIt + RViz
rosrun agx_xarm_kinematics marker_ik_teleop.py _enable_arm:=false
#    RViz: 拖动 marker（arm_display.rviz 已含 InteractiveMarkers）

# D. 真机（清场、急停在手、确认低速度后）
rosrun agx_xarm_kinematics marker_ik_teleop.py _enable_arm:=true
```

验收清单：

- [ ] `--check-urdf` 逐段一致、`--check-jac` < 1e-6
- [ ] 离线路径 0 越限（位置/速度/加速度三张图全在限制线内）
- [ ] 干跑：marker 收敛后贴 `tcp_link`，日志 `||p−p_des||` 收敛 < 1 mm
- [ ] 真机：低速度平滑跟随，无跳变；`~stop`/急停有效

---

## 6. 风险与未决项

| # | 风险/未决 | 应对 |
|---|---|---|
| 1 | ~~M2 `tcp_link` 0.230 m 存疑~~ **已解决**：2026-09-29 复测为 0.170 m | FK 不硬编码任何 TCP 长度；`--check-urdf` 与 URDF 现值强绑定；再复测改 URDF 即自动跟随 |
| 2 | 升降不闭环（仿真自由度） | 真机模式强制 `lift_mode=locked`，代码级拒绝 `free` + `enable_arm` 组合 |
| 3 | 与 MoveIt/滑条抢同一个控制器 | 运行期提示互斥；文档规定 teleop 期间不用其它控制面板 |
| 4 | 拖动目标不可达 | 目标裁剪盒 + 残差可视化（红球）/日志 |
| 5 | RViz 拖动中 setPose 抢鼠标 | `DRAGGING` 状态不 setPose（3.3） |
| 6 | 反馈噪声导致盒子冲突 | 退化 hold（2.3 第 3 点），不放大动作 |
| 7 | 解答辩/演示时长过长 | 参数表在 YAML，现场只调 `rate_hz`/`vmax`/`amax` |

---

## 7. 与已确认决策的对应

| 决策 | 落点 |
|---|---|
| "气泵"=升降柱 lift_joint | 1.2 第 1 段 / 1.6 / 2.7 |
| 7 自由度一起解，但只仿真 | `lift_mode: free`（离线/dry-run）；实机 `locked`（2.7） |
| 位置 + 保持启动姿态 | 2.1 的 `w_o` + 启动时固定 `R_des` |
| 代码放 `src/core/agx_xarm_kinematics` | 第 4 节 |
