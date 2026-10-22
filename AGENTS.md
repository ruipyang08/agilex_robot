# AGENTS.md

本文件面向在此工作空间中工作的 AI Agent / 开发者，用于快速理解项目结构与约定。
修改代码前请先阅读本文件。

## 1. 项目概述

这是一个 **ROS1 (Noetic) catkin 工作空间**，面向 AgileX 移动操作机器人：

- 移动底盘：AgileX Ranger Mini（CAN 总线，`ugv_sdk`）
- 升降柱：lift module（`lifting_ctrl` 服务控制）
- 机械臂：UFACTORY xArm6（MoveIt 规划）
- 末端：DH Robotics AG95 两指夹爪
- 感知：RoboSense 雷达（`rslidar_sdk`）、Intel RealSense D435、CH110 IMU、AR 码识别

主链路：

- 导航：雷达 -> `pointcloud_to_laserscan`(`/scan`) -> `rf2o`(`/odom`) -> gmapping/move_base -> `/cmd_vel` -> 底盘
- 机械臂：MoveIt(`xarm6` 规划组) -> xArm 轨迹控制器 -> xArm 硬件服务
- 抓取：AR 码位姿 + MoveIt + 夹爪，由 `agx_xarm_pick` demo 编排

注意：根目录**不是**单一 git 仓库，很多子包是带有各自 `.git` 的上游第三方仓库。

## 2. 环境与工具链

- OS：Ubuntu 20.04，ROS1 **Noetic**
- Python：3.8（有 Python2 风格残留，新代码用 Python3）
- 构建：`catkin_make`（非 catkin_tools），CMake `Release`
- 终端使用前先 source：

  ```bash
  source /opt/ros/noetic/setup.bash
  source ~/agilex_ws/devel/setup.bash
  ```

### SSH 远程使用（重要）

通过 SSH 连接机器人时，`$DISPLAY` / `$XAUTHORITY` 默认为空，`setup_arm.launch` 中的
`joint_slider_gui`（Qt）和 RViz 会因 `could not connect to display` 崩溃。它们崩溃**不会让
roslaunch 退出**，但界面出不来，容易被误判为“无法启动”。

已在机器人 `agilex` 用户的 `~/.bashrc` 中加入（仅 SSH 登录且无 DISPLAY 时生效）：

```bash
if [ -n "$SSH_CONNECTION" ] && [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
    export XAUTHORITY="$HOME/.Xauthority"
fi
```

- 新开的 SSH 终端自动生效；已在运行的终端需 `source ~/.bashrc`
- 图形窗口显示在**机器人本地显示器**上，不会出现在 SSH 客户端
- 想让界面显示在自己电脑：`ssh -X`（此时 DISPLAY 已存在，上面的判断不会覆盖它）

## 3. 目录结构

```text
agilex_ws/
├── src/
│   ├── car_base/         # 底盘：ranger_ros(ranger_base/bringup/msgs/sensor_msgs) + ugv_sdk
│   ├── core/             # 应用逻辑：agx_xarm_pick(抓放 demo)、agx_xarm_smach(任务状态机)
│   ├── msg/              # 消息/服务：agx_pick_msg、ar_track_alvar_msgs、dh_gripper_msgs
│   ├── moveit_config/    # 机器人模型与 MoveIt 配置（见下）
│   ├── plugin/           # RViz 多点导航插件
│   ├── sensors/          # camera / gripper / imu_ch110 / lidar 驱动
│   ├── SLAM/             # gmapping、rf2o、laser_scan_matcher、pointcloud_to_laserscan
│   ├── start/            # 集成 bringup：agilexpro、agx_xarm_bringup、lifting_ws(子工作空间)
│   ├── tasks/            # 任务层：agx_plane_target(平面红点选点+MoveIt到位)、doc/(决策/实测/设计稿)
│   └── easy_handeye/     # 手眼标定（第三方）
├── build/  devel/        # catkin 产物（不要手改、不要提交）
├── README.md             # 详细上手指南（架构/操作手册/排障），首选参考
├── WORKLOG.md            # 逐日工作日志（项目进展/答辩参考，最新在上）
├── map.pgm / map.yaml    # 现场地图
├── frames.gv / .pdf      # TF 树导出
└── MarkerData_*.png      # AR 码图
```

### moveit_config 关键包

| 包 | 说明 |
|---|---|
| `ranger_xarm_description` | 整机组合模型 xacro（`ranger_xarm6_lifting.xacro` 等），含 xArm6/升降/夹爪 mesh |
| `ranger_xarm_base` | 底盘-升降转接结构 URDF/mesh |
| `ranger_xarm_lifting_moveit_config` | **当前实际使用**的 MoveIt 配置（Ranger+升降+xArm6），入口 `launch/realMove_exec.launch`，SRDF 规划组 `xarm6` |
| `ranger_xarm_moveit_config` | 无升降版本的 MoveIt 配置 |
| `c1997` | 升降柱结构描述 |
| `dh_robotics_ag95_model` | AG95 夹爪模型 |
| `ranger_mini` / `ranger_mini_v3` / `ranger_mini_v3_gazebo` | 底盘描述与仿真 |
| `scout_xarm/` | Scout+xArm 参考工程，内嵌官方 `xarm_ros` 源码树；**非当前机型，勿随意改动** |

## 4. 高频入口

| 功能 | 命令 / 文件 |
|---|---|
| 机械臂（含 MoveIt/RViz + 控制台） | `roslaunch agx_xarm_bringup setup_arm.launch`（默认同时启动升降节点，可用 `open_lift:=false` 关闭） |
| 夹爪 | `roslaunch agx_xarm_bringup open_gripper.launch` |
| 相机 / AR 码 | `roslaunch agx_xarm_bringup open_camera.launch` |
| 抓取 demo | `rosrun agx_xarm_pick demo_pick`（默认 marker ID 0） |
| 底盘 | `roslaunch ranger_bringup ranger_mini_v2.launch` |
| 雷达 | `roslaunch agilexpro open_lidar.launch` |
| 建图 | `roslaunch agilexpro gmapping.launch` |
| 导航 | `roslaunch agilexpro navigation_4wd.launch` |
| 升降 | `roslaunch lifting_ctrl start_motor.launch`，然后用 `lift` 命令（见下）或 `rosservice call /lifter_1/LiftingMotorService ...` |
| 平面红点选点（拖动/方向键 + MoveIt 到位） | `roslaunch agx_xarm_bringup setup_arm.launch` 自带（**默认点一下就执行**）；用法见本节「平面红点选点与到位」 |
| 放入点（六点，终端） | `put <1~6>`（脚本 `src/tasks/agx_plane_target/scripts/put`；先 `put install` 可让 `put1..put6` 直接可用）；用法见本节「放入」 |

### 升降柱控制（830ABS）

- 机型 **830ABS**（驱动器正面有标签；**不要**用 850pro 的 launch/协议）
- 串口：`/dev/ttyUSB2`（CH340）；节点会自动扫描 `/dev/ttyUSB*`，夹爪占用的 `/dev/ttyUSB1` 会被跳过
- 启动：`roslaunch lifting_ctrl start_motor.launch`（**独立于机械臂**，可单独运行）
- 服务：`/lifter_1/LiftingMotorService`；状态话题：`/lifter_1/LiftMotorStatePub`
- **高度 0~600mm，0 = 最低，600 = 最高**（值越大越向上，约 11mm/s，全程约 55 秒）
- ⚠️ **整机电断电后位置基准会丢失**（2026-09-29 实测推翻旧说法）：830ABS 的位置基准靠
  「下降压下限位 + 清零」建立，**整机断电重启后丢失**，驱动器读数回到 ≈0 而柱子物理上还在
  行程中间（实测 `backPos=-206` / `backHeight=0` 但 `downLimit=False`）。原先这里写的
  「绝对值编码器，`initState` 会保留，无需重新初始化」**是错的**——`initState` 只是软件标志
  （`mode != 1` 就置 True），和驱动器位置基准是否还在毫无关系。**每次整机电断电后，
  开机第一步必须 `lift init`**（真实运动，约 1 分钟，下降到下限位重建基准）
- **坐标系未标定时会自动锁死**（`decisions.md` D20）：节点把可信高度落盘
  `~/.ros/lift_frame_m1.json`，上电比对差 >20mm 判为未标定 → 拒绝位置模式**和所有速度模式**
  （含下行，因为速度模式会一直跑到撞机械限位、没有自动退出）返回 `-14`、50Hz 定位回路不下发位移、
  对下游发布 `initState=False`；控制台升降柱那一行**置灰并显示「未标定」**，
  `lift <高度>` 和 `lift up/down` 都被脚本拒绝，`lift status` 明确打出「不可信」。
  `lift init` / `lift stop` / `lift` 的 `init`/`stop` 仍可用，`lift init` 跑完自动恢复并落盘
- 便捷命令 `lift`（脚本 `~/bin/lift`，`$HOME/bin` 已在 PATH；脚本自动 source ROS 环境）：

  ```bash
  lift status        # 查看当前高度/限位/过载等状态
  lift 300           # 升/降到 300mm 并等待到达（显示进度）
  lift 0             # 回最低点
  lift up 20         # 以 20mm/s 上升；lift down 20 下降
  lift stop          # 急停
  lift init          # 初始化（会下降到下限位，需输入 yes 确认）
  lift help
  ```

- **控制台集成（重要）**：`joint_slider_gui`（窗口标题「控制台」，由 `setup_arm.launch` 启动）内置「升降柱」滑条：

  - 拖动时**实时下发**高度（限流 10Hz），松手时强制下发最终值
  - 未拖动时滑条跟随实际高度；右侧显示 `已到达 / 运行中 / 未初始化 / 故障 / 未连接`
  - 自带「急停」按钮（发送 `mode=-2`）
  - 升降节点未运行时该行自动置灰，不影响机械臂控制
  - 范围从 `LiftMotorStatePub` 的 `upLimitHeight/downLimitHeight` 动态读取
  - 源码：`src/start/agx_xarm_bringup/src/joint_slider_panel.cpp`（依赖 `bt_task_msgs`）

- 服务原始用法：`rosservice call /lifter_1/LiftingMotorService "{val: 300, mode: 0}"`

  | mode | 含义 | val |
  |---|---|---|
  | `0` | 位置控制（常用） | 高度 mm（0~600） |
  | `-4` | 速度控制 | mm/s，正=向上 |
  | `-2` | 急停 | — |
  | `1` | 初始化（下降到下限位） | — |
  | `-3` | 清除错误（不动作） | — |
  | `-5` | 设定位置模式速度 | rpm |
  | `-6` | 位置清零（**慎用**，会改变零点） | — |

  返回 `state`：`1` 到达、`-1` 未到达、`2/-2` 触碰限位、`-14` 未初始化/坐标系未标定、`-12/-13` 超时(自动急停)、`-9` 电机离线。

- 配置文件：`src/start/lifting_ws/src/lifting_ctrl/scripts/config/1_lifting_motor_config.json`
  - `initMode`：`1` = 节点启动即自动下降到下限位初始化；`0` = 不自动初始化（**当前为 0**；改 1 会开机无人值守地自动下降，不采纳）
  - `upLimitVal` 600 / `downLimitVal` 0 定义软件行程
  - 门禁参数在节点上（不重编译可调）：`~frame_check`(true) / `~frame_tolerance`(20 mm) / `~frame_file`
- **升降柱不在 MoveIt 规划组内**，升降动作仍走独立的 `lifting_ctrl` 服务，MoveIt 不规划、不驱动它；但 URDF 里有 prismatic 关节 `lift_joint`（`ranger_xarm6_lifting_2431.xacro`，行程 0~0.6 m），`lift_joint_bridge.py`（`setup_arm.launch` 启动，只读）把实际高度喂给 MoveIt，**机械臂规划/碰撞检查/TF 已随升降高度实时变化**。升降柱自身与升降过程中的臂位仍需人工确认空间。
- **3 个固定参考系 + 1 个运动系，别混用**：`link_base` 随升降移动（z = 0.555 + lift/1000）；`link_base_lift0` 是 `setup_arm.launch` 里静态 TF 给出的 lift=0 快照（在 `base_link` 系恒为 (-0.19, 0, 0.555)）；`deck_top` 是 2431 箱体顶面（升降柱安装平台，**卡尺实测** lift0 下方 208mm，`base_link` 系恒 (-0.19, 0, 0.347)、离地约 667mm）。需要「机械臂底座固定参考位」用 lift0，需要「底盘上层平台面」用 deck_top，需要「当前实际底座位姿」用运动的 link_base。RViz Marker 可视化（`setup_arm.launch` 自动启动，仅显示）：绿盒 212×212×650 mm（`/lift_zone/marker`）+ 红盒臂底座 155×126×155 mm 挂在运动的 `link_base` 上；黄色暂定安全区（`/safety_plane/marker`，`safety_plane_viz.py`）底面 = `deck_top` 上方 **28mm**、向上到 1.298m（高 1.27m，2026-09-29 从底面 98mm 下调 70mm 以便夹爪更靠近甲板），截面 y ±0.25 罩住底盘、x 已切掉 lift0 系 x≤106mm（柱前缘）只留柱前区，静止。绿盒底部是包围盒下沿（lift=0 时离地 225mm），**不是**底盘顶面，别用它反推平台位置（见 `decisions.md` D18）；安全区为暂定值（D19）。

### 平面红点选点与到位（`agx_plane_target`）

在黄色暂定安全区**底面**（`deck_top` 上方 100mm，即盒内 z=0.100；盒底 2026-09-29 下调到 28mm，红点仍固定在 100mm）放一个可拖动的红点，点一下就让夹爪竖直朝下走到该点，
MoveIt 负责规划（避障）。源码 `src/tasks/agx_plane_target/scripts/plane_target_marker.py`（任务层包）。

- **启动**：`setup_arm.launch` 默认带上，参数 `open_plane_target:=false` 可关掉

  ```bash
  roslaunch agx_xarm_bringup setup_arm.launch                              # 默认直接执行（点一下就动）
  roslaunch agx_xarm_bringup setup_arm.launch plane_target_execute:=false  # 干跑：只规划不动机械臂
  roslaunch agx_xarm_bringup setup_arm.launch open_plane_target:=false     # 不用红点
  ```

- **执行开关**：`~enable_arm`（launch arg `plane_target_execute`，**默认 `true`** = 规划并执行）**只在节点启动时读一次**，
  运行中改无效。启动时节点会打一条 `logwarn` 明示当前模式：`enable_arm=true: GO will MOVE the real arm...`
  或 `enable_arm=false: dry-run...`。其它参数（都在节点标签内加 `<param/>`）：`~vel_scaling`(0.2)、`~planning_time`(10.0)、
  `~plane_z`(0.100)、`~rpy_deg`("180,0,0" = tcp **+Z**（AG95 进给轴）竖直向下；**每次 GO 实时读取**，
  `rosparam set /plane_target_marker/rpy_deg "180,0,90"` 拧向不用重启)
- **朝向自检**：每次 GO 会打「命令姿态 + 工具轴离竖直方向多少度」，执行完再回读 TF 打
  「实际三根轴 + 离竖直多少度 + 与命令点差几 mm」。**但 TF 由 `joint_states` 推出，天然与命令一致**，
  日志里倾角 ≈0° 只证明"命令到位"，实物是否竖直必须现场用水平尺/气泡水平仪复核——
  有偏差说明是姿态相关的关节零位问题，不是规划 bug
- **轨迹安全区约束**（`~keep_in_zone` 默认 `true`）：末端**在整个运动过程中**都必须留在黄色安全盒
  （`deck_top` 系 `x[0.106,0.565] y[±0.250] z[0.028,1.298]`）内——用 MoveIt path constraint 实现
  （`PositionConstraint.constraint_region` 里放 `BOX` primitive），MoveIt 每个采样状态都检查，所以限制的是
  整条扫掠轨迹而不只是终点。
  **盒子数字的单一真源 = `safety_plane_viz.py` 的 `PLANE`**：它启动时把 `{center, size, frame_id, ns}`
  写进参数 `/safety_plane/box`，`plane_target_marker.py` 每次运动重读（读不到才退回内置兜底常量并
  `logwarn`）。要改盒子**只改那一个文件**，没有第二份拷贝要同步。记法：`底面 = pos.z − size.z/2`；
  想降底面 d 而顶面不动 → `size.z += d`、`pos.z -= d/2`。
  参数：`~zone_margin`(0.0，六面内缩；**红点在 z=0.100、盒底 0.028，内缩 >72mm 红点就不可达**)、
  `~zone_links`("tcp_link"，可加 `link6`/`link5` 收紧；**不能约束整条臂**——臂基座在 x=0 而盒子从 x=0.106 起)、
  `~home_named_state`("initial")。**只约束末端**：关节滑条手动拖动是直接下发的，加不了这种约束。
  MoveIt 路径约束对**起点也生效**，所以节点会在被约束 link 位于区外时把盒子最小化扩一个包住它并 `logwarn`，
  避免"规划莫名超时"。每次 GO 会打印实际生效的 bounds 和盒子来源
- **RViz 操作**：红点**用鼠标直接拖动**（marker 挂 3 个 control：`drag_plane` 45mm 红球 +
  `grab_handle` 100mm 半透明球加大抓取目标 + `hint` 不可拾取的操作提示文字；两个拖动 control 都是
  `MOVE_PLANE`，**不要再叠 `BUTTON`**），在黄区内**平面拖动**（越界自动钳位回矩形）；
  **短按红点**（<0.5s 且位移 <5mm）= 执行。控制台「← → ↑ ↓」方向键同时保留、两条路等价。
  颜色：待命红 / 运动中黄 / 成功绿 / 规划失败橙（2s 回红）。**RViz 里没有急停控件**（初版的红色方块已移除，
  急停统一由控制台「急停」按钮 + `plane_target/stop` 服务承担）
- **控制台操作**（`joint_slider_gui`，升降柱行下方「平面红点」一行）：`← →` = ∓X（车尾/车头）、
  `↑ ↓` = ±Y（左/右）、步长 1~200mm（默认 20mm）、「回中心」、「执行 GO」、「急停」。
  位置标签与 RViz **双向同步**。GO 前会检查机械臂状态（急停锁住/拖动示教会拒绝）。节点没起时整行置灰
- **控制台「夹取」行**（在「平面红点」下方）：`闭合位置` 输入框（0~1000，默认 0）+
  「夹取（移到红点后闭合）」按钮 + 状态标签（夹取中/已完成/失败/已拒绝）。
  执行期间按钮**自动禁用**（防连点把两条指令塞进同一个 `xarm6` 控制器），
  结束后**自动重新同步夹爪滑条**（夹取直接改了夹爪位置，滑条否则会显示旧值、下次拖动像失控）。
  详细读数看窗口最上面那行状态文字
- **数值**（`deck_top` 系，与 `safety_plane_viz.py` 同源）：矩形 x[0.106, 0.565]、y[±0.25]、z=0.100，初始点取中心 (0.3355, 0)
- **接口**（脚本/状态机可直接用）：

  | 名称 | 类型 | 方向 | 说明 |
  |---|---|---|---|
  | `plane_target/target` | `geometry_msgs/PoseStamped` | 发布（latched） | 当前红点位姿（deck_top 系） |
  | `plane_target/nudge` | `geometry_msgs/Vector3` | 订阅 | x/y = 相对步进（米） |
  | `plane_target/go` | `std_srvs/Trigger` | 服务 | 规划+执行（等同短按红点） |
  | `plane_target/stop` | `std_srvs/Trigger` | 服务 | 取消当前运动 |
  | `plane_target/home` | `std_srvs/Trigger` | 服务 | 规划+执行回 SRDF 命名位姿（`initial`），**带安全区约束**；控制台「回到初始位置」优先调它，没有它才退回旧的直发关节轨迹（无规划无检查） |
  | `plane_target/reset` | `std_srvs/Trigger` | 服务 | 红点回矩形中心 |
  | `plane_target/grasp` | `std_srvs/Trigger` | 服务 | **夹取（一个原子动作）**：(可选张开) → MoveIt 移到红点 → 闭合到 `~grasp_position` → 回报实际停住位置。控制台「夹取」按钮调用，见下 |
  | `plane_target/grasp_status` | `std_msgs/String` | 发布（latched） | 夹取进度，格式 `"<state>\|<text>"`，state ∈ `busy`/`ok`/`fail` |
  | `plane_target/put` | `agx_plane_target/PutPoint` | 服务 | **放入（一个原子动作）**：请求带点号 1~6（M6 顺序），五步 = 上方点 → 直线下降 → 松开 → 直线抬高 → 回 `initial`；终端 `put <1~6>` 调用，见下 |
  | `plane_target/put_status` | `std_msgs/String` | 发布（latched） | 放入进度，格式同上（**独立话题，控制台不订阅**） |
  | `plane_target/put_points` | `visualization_msgs/MarkerArray` | 发布（latched） | 六个放入点的**编号红球**（frame `link_base_lift0`），纯显示 |

- **夹取（`plane_target/grasp`，2026-09-30 新增，决策见 `decisions.md` D21）**：控制台「夹取」那一行
  = 一个「闭合位置」输入框（0~1000）+ 一个「夹取（移到红点后闭合）」按钮 + 一个状态标签。
  - **一次点击 = 移到红点 + 闭合**，不是"先点 GO 再点闭合"。因为用户要在此基础上做自动化流程，
    **自动化的原子单位必须是服务**；且整个序列期间 `busy` 保持为真，防止 GO/home 插进
    "移动完了还没闭合"的窗口把半闭合的夹爪抬走
  - **闭合位置可输入调节**：`0` = 尽量闭合、靠力度顶住物体（各种尺寸都能夹，推荐默认）；
    夹不动/易碎/已夹着的物品调大，让手指提前停。**只暴露位置，不暴露力度**（用户要求）；
    力度固定 50，与夹爪滑条一直发的一致，要改用 `rosparam set /plane_target_marker/grasp_force 30`
  - 参数（`~grasp_position`/`~grasp_force`/`~grasp_speed`/`~grasp_preopen`/`~grasp_settle_*`）
    **每次夹取重读**，换物品只改数字、不重启任何节点。控制台点按钮前先
    `nh_.setParam("/plane_target_marker/grasp_position", v)` 再调服务
  - **到位判据 = 位置读数连续 N 次不再变化**，**不读 `grip_state`**（M4 至今没实测，猜它等于把
    猜测写进安全判断）。有 0.25s 起步窗口（指令刚发出手指还没动，那段"静止"是假的）+ 3s 超时，
    超时**如实报出实际停在多少**，不谎报成功
  - **不判断"夹住 vs 夹空"**，只报实际停住位置（停在目标值=大概率空夹，停在比目标大=大概率夹到）
  - **`enable_arm=false`（干跑）时只规划、绝不闭合夹爪**——干跑的语义是"不碰真机"，闭合也是碰真机
  - **急停能中断闭合**：AG95 没有 cancel 输入，实现方式是**重发它已到达的位置**（保持）
  - 节点没起 / 版本旧 → 整行自动置灰（松耦合字符串接口，`start` 不依赖 `tasks`）

- **放入（`plane_target/put`，2026-10-14 新增，决策见 `decisions.md` D22，示教点见 `measurements.md` M6）**：
  把手里夹着的物体放到六个示教点之一。**接口先做终端命令**：`put <1~6>` / `put3`
  （`src/tasks/agx_plane_target/scripts/put`；`put install` 把 `put` 和 `put1..put6` 链接进 `~/bin`，
  脚本自己补 ROS 环境；`put status` 看进度、`put stop` 急停，等同 `/plane_target/stop`）。
  **控制台没有对应按钮**（按需求不与控制台耦合）。
  - 一次调用走完五步：**到该点上方 100 mm → 直线下降 100 mm → 松开(1000) → 直线抬高 → 回 `initial`**；
    全程 `busy` 占住（期间 GO/夹取/再来一次放入都会被服务拒绝）。调用前要求手臂在 `initial`、物体已夹住；
    服务**不会先回家**（避免提着物体从低位扫过甲板）
  - 六个点在 RViz 里是**编号红球**（`plane_target/put_points`，MarkerArray，点号 = marker id，
    `+100` 是数字标签，frame `link_base_lift0`）；`arm_display.rviz` 已加显示项。红球**只是显示、不可拖**；
    要重示教先改 `measurements.md` M6，再同步 `plane_target_marker.py` 的 `PUT_POINTS` 常量
  - 坐标在**静止系 `link_base_lift0`**；**每次执行**用当时的 TF 变到当前的 `link_base` 再规划
    （升降高度变了不必重教这六个点）
  - **竖直两段是 Cartesian 直线**（`compute_cartesian_path`，碰撞检查保留）且**故意不套黄区约束**
    （示教点可能在黄盒外，套上直线下降会规划失败）；另两段照常带黄区约束，并**把约束盒底面抬到
    放入点高度**（"不回到放入高度以下"）。竖直两段检查 `fraction`（不完整不执行），并按 `~vel_scaling`
    客户端重定时后才执行（MoveIt 的 Cartesian 服务固定按全速时间参数化）
  - 进度只发 `plane_target/put_status`（**独立话题**，控制台「夹取」行不受影响）
  - **`enable_arm=false`（干跑）只规划第 1 步「到上方点」**，并明说下降/松开/抬高/回家未执行
    （不动机械臂就没法从"假设的上方点"继续规划直线段，所以不假装规划过）
  - 失败/急停一律如实报**停在哪、物体是否还夹着**；判据沿用夹取的"位置停止变化"，不读 `grip_state`
  - ⚠ 操作提醒：put 期间**别按控制台「回到初始位置」**——它虽然会被 `busy` 拒绝（只显示"运动中，
    请先停止"），但按钮按下时**先发一条夹爪张开指令**，会把手里/正在放的物体松开；也别拖夹爪滑条
    （手动夹爪控制一直在线，与本节点无关，`busy` 拦不住它）

- **坐标系要点**：红点挂在**静止系 `deck_top`**（不随升降动）；**点击瞬间**做一次 `deck_top → link_base` 变换把目标固化，
  之后升降被改动也不影响这次运动。末端用 `set_end_effector_link("tcp_link")`（**不是** SRDF 组末端 `link6`）
- **急停语义**：`group.stop()` 取消 MoveGroup 目标，机械臂**停在当前位置并保持上电**，**物理急停才是断电停止**。
  与关节滑条/拖动示教共用同一个 `xarm6` 控制器，运动期间不要同时操作
- **安全须知（默认就是真机执行，点之前务必确认）**：清场（臂前/臂下 1m 内无人和障碍）→ 手边物理急停 →
  先用方向键把红点挪到离当前夹爪 5~10cm 内验证方向 → 再去远处目标 → 想先看路径用 `plane_target_execute:=false` 干跑
- 与 `src/tasks/doc/kinematics_ik_plan.md` 是**两套独立方案**：本套是"离散选点 + MoveIt 规划"，
  那套是"连续拖动 + 30Hz 逐拍 IK 限幅"（未实施）

## 5. 构建

```bash
source /opt/ros/noetic/setup.bash
cd ~/agilex_ws
catkin_make                       # 全量构建
catkin_make --pkg <pkg_name>      # 单包构建
source devel/setup.bash
```

- 首次或改了 `package.xml`/消息后建议全量构建。
- **新增包后第一次构建要用全量 `catkin_make`**：`catkin_make --pkg <新包>` 会报
  `The specified source space ... does not exist` 或 `build/<pkg>` 找不到（此时 CMake 还没登记这个包）。
  之后再单包构建就正常了。
- `src/start/lifting_ws` 是内嵌子工作空间，但其中的 `lifting_ctrl` / `bt_task_msgs` 同时被外层构建（产物在 `devel/share/lifting_ctrl`）。
- 构建报 "package not found" 时，先确认已 source 且路径正确：`rospack find <pkg>`。

### 打包前闸门（2026-10-14 真机 feedback 后新增，必做）

要打包上真机的源码，提交前必须通过 offboard 部署契约（
`docs/deployment/offboard_deployment_contract.md`，规则 A–H）体检。offboard 侧用
`scripts/deploy_check.py` + clean 全量 `catkin_make` 复现真机构建条件：

```bash
# 仓库根（offboard）：
python scripts/deploy_check.py                 # FAIL 非零 → 禁止打包（WARN 为记录在案例外）
python scripts/build_ros1_workspaces.py --profile hardware   # 强制 clean catkin_make，EXIT 必须为 0
source agilex_ws/devel/setup.bash
bash agilex_ws/src/tasks/tests/run_all.sh      # 必须全绿
```

硬规则速记：A `package.xml` 与 `find_package(catkin COMPONENTS ...)` 一致；B 消费消息
的 target 必须有**未注释**的 `add_dependencies(... ${catkin_EXPORTED_TARGETS})`；
C `src/**` 禁止硬编码 `/home/<user>/...`；D 顶层 launch 默认参数可解析且每模式最多一个
RViz；E ≥1 Hz 周期打印默认关闭；F 打包前必须 clean `catkin_make`（增量不算数）；
G 打包绝不包含生成物；H 真机标定参数（相机序列号/升降标定/安全区）以真机为准。
打包脚本已集成该体检：`package_agilex_workspace.py` 出包前自动跑 `deploy_check`，
FAIL 会拒绝出包。

> 完整流程准绳（改码顺序 / 记录纪律 / 真机回程闭环 / 通过判据表）见
> `docs/development/modification_guidelines.md`，功能修改前先读它。

## 6. 开发规范与约定

- **ROS1 风格**：包结构遵循 `package.xml` + `CMakeLists.txt` + `launch/` + `config|param/` + `src|scripts/` + `rviz|meshes|urdf/`。
- **命名**：自研包前缀 `agx_`；话题用 `/cmd_vel`、`/odom`、`/scan`、`/joint_states` 等惯用名。
- **消息**：改动 `.msg/.srv` 后必须重新 `catkin_make` 才能被其他包使用。
- **配置外置**：参数放 YAML（`param/`、`config/`），不要硬编码到源码。
- **不要手改生成物**：`build/`、`devel/`、`*.pyc`、`__pycache__/`、`*.back`、`export.log`、`temp.txt` 等。
- **第三方代码**：`ugv_sdk`、`easy_handeye`、`realsense-ros`、`velodyne`、`rslidar_sdk`、`ar_track_alvar`、`gmapping`、`pointcloud_to_laserscan`、`scout_xarm/xarm_ros` 尽量不改；确需修改要说明原因。
- **改动最小化**：与现有风格保持一致，优先改 launch/YAML 而非重写驱动。
- **分层依赖方向（硬规则，见 `src/tasks/README.md`）**：`tasks → core/start/msg/moveit_config`，
  **`core`、`start` 禁止编译依赖 `tasks`**。跨层调用（如 `joint_slider_panel.cpp` 调平面红点）只用
  话题/服务名的**松耦合字符串接口**，并在调用方做"对方没起就置灰/降级"。
- **安全**：任何会驱动真实硬件的命令（底盘/升降/机械臂/夹爪）都需人工确认，机器人周围清场。
- **会动硬件的功能要有显式开关和醒目提示**：默认行为由负责人决定（平面红点按 2026-09-28 决定改为
  **默认直接执行**，靠控制台「急停」+ 物理急停兜底），但无论默认什么，都要有
  `~enable_arm` 这类参数 + launch arg 能在**不重编译**的前提下关掉（它只在启动时读，改了要重启节点），
  并在启动日志里 `logwarn` 明示当前是"会动"还是"干跑"。
- **离线测试**：`bash src/tasks/tests/run_all.sh`。`src/tasks/tests/` 下是纯逻辑测试
  （升降门禁 / `MotorStop` 限位 / bridge 持久化 / 夹取 / 放入），**不碰串口、不碰真机、
  不需要 roscore、落盘走临时目录**，可以在机器人运行时安全执行，不会和 `setup_arm.launch`
  抢节点名。仍需 source ROS 环境（要用 `bt_task_msgs` / `sensor_msgs` / `dh_gripper_msgs`）。
  `test_grasp.py` / `test_put.py` 连 MoveIt 规划都换成了桩，所以不会抢 `xarm6` 控制器。
  2026-10-14 实测（容器，真机同款 Python 3.8 + 真消息）：**56 PASS + 162 ok / FAIL 0**
  （test_put 贡献 102 条 ok）；更早记的 116 是升降/夹取那一批。桩的写法要两台环境都能跑：
  `staticmethod` 包装在 3.8 上不可调用（3.10 起才可以），别用。

## 7. 常见坑（务必注意）

1. 包名是 **`agx_xarm_bringup`**，不是 `agx_arm_bringup`。
2. **TF 帧名非标准**：常见 `world`、`odom`、`dummy_base_link`、`rslidar`、`base_link`、`link_base`、`link6`，不要假设只有 `map->odom->base_link`；调试用 `rosrun tf view_frames`。
3. **`/odom` 有两套来源**：纯底盘时看 `/ranger_base_node/odom`；雷达导航时 `/odom` 由 `rf2o_laser_odometry` 发布。
4. **MoveIt 规划组只有 `xarm6`（6 自由度）**，升降不在规划组内，走独立的 `lifting_ctrl` 服务；想联动作业需单独处理。
5. 地图/参数在不同 launch 中可能使用 `world` 而非 `map` 作为 map frame。
6. 雷达/相机/IMU 需要正确 USB/CAN 与权限（`ttyUSB_permission.rules`、`setup_can2usb.bash`）。
7. 根目录 `README.md` 是最完整的背景资料，遇到不明确处优先查它。
8. **SSH 下 GUI 崩溃**：`joint_slider_gui` / RViz 报 `could not connect to display` 时，见第 2 节「SSH 远程使用」（已在 `~/.bashrc` 自动处理）。
9. **升降柱方向与初始化**：高度 `0` = 物理最低、值越大越向上（0~600mm）；`initMode=1` 会让节点启动时自动下降到下限位，属真实运动，务必先确认下方空间。
   **另外：整机电断电后位置基准丢失，开机必须先 `lift init`**（见第 4 节和 `decisions.md` D20）。
   症状是控制台显示「已到达 0mm」——旧版本这里不会锁死，可以直接拖滑条，会撞机械硬限位。
   看到「未标定」就 `lift init`，不要试图用 `lift 0` 或拖滑条去"归零"。
10. **launch 注释里不能出现连续双连字符 `--`**：会直接让整个 launch 解析失败
    （`RLException: Invalid roslaunch XML syntax: not well-formed (invalid token): line N, column M`）。用 `/` 或分号代替。
11. **RViz `InteractiveMarkers` 显示项的属性名是 `Update Topic`**，没有 `Interactive Markers Namespace`。
    写错的键被**静默忽略**，marker 一个都不显示——改完 `.rviz` 一定要实际看一眼。
12. **`InteractiveMarkerControl.always_visible` 默认 `false`**：此时 RViz 只在鼠标悬停该 control 时才画它的内容，
    marker 会"完全看不见"。拖动球/按钮类 control 一律显式设 `true`。
13. **同一个 interactive marker 上不要叠同位置的 `MOVE_PLANE` + `BUTTON` 两个 control**：RViz 拾取顺序不确定，
    拖动和点击互相抢。可靠做法是单一 `MOVE_PLANE` + `MOUSE_DOWN`/`MOUSE_UP` 短按判定（见 `plane_target_marker.py`）。
14. **RViz 退出时别误存配置**：RViz 弹"保存配置"如果确认保存，会覆盖手工修过的 `.rviz` 文件。
    ⚠️ 更隐蔽的一层：**RViz 存配置时会把"当前激活的工具"写到 `Tools:` 列表首位**，所以万一在用
    「移动相机」时误存一次盘，下次启动就是移动相机激活 → 鼠标拖动只转视角、interactive marker
    （平面红点）**完全没反应**，且极易被误判成"marker 代码有问题"。`arm_display.rviz` 就这样踩过
    （工作区 40+ 个 .rviz 里只有它首位是 `rviz/MoveCamera`）。排查"RViz 里 marker 看得到但点不动"
    时**先看 `Tools:` 首位是不是 `rviz/Interact`**；临时补救不用重启——点工具栏最左边的 Interact
    （手掌图标）按钮即可。
15. **删掉代码里的 interactive marker，RViz 里那个不会自动消失**：RViz 对已停止节点的 marker 永不过期；
    而 `InteractiveMarkerServer.erase()` 对本进程没创建过的 marker 是**静默空操作**（源码里取不到上下文就
    `except: pass`）。要清掉已废弃的 marker，得在启动后一段时间内 1Hz 重复发
    `InteractiveMarkerUpdate(erases=[名字])`（见脚本里的 `RETIRED_MARKERS` / `RETIRE_ERASE_GRACE_S`），或直接重启 RViz。
16. **SRDF 的 `xarm6` 组末端是 `link6`，不是 `tcp_link`**：要按"指尖中点"下指令必须显式
    `set_end_effector_link("tcp_link")`（URDF 里 `tcp_link` 是 `link6` 下方 **0.170m** 的纯坐标系，不参与碰撞；实测标定值，见 `measurements.md` M2）。
17. **验证时不要手工重复起 static TF**：`base_link→link_base_lift0`、`link_base_lift0→deck_top` 已由
    `setup_arm.launch` 发布，再起同名广播器会和它打架。测试要用独立 `roscore` 的话先确认用户环境状态。
18. **`tf.transformations` 的欧拉角函数吃弧度，不吃度**：`quaternion_from_euler(180, 0, 0)` 不是"绕 X 转 180°"，
    而是 180 **弧度** = 233.3°，**不报错、只是安静地算出另一个合法姿态**。凡是参数名带 `_deg` 的，
    进欧拉角函数前必须 `math.radians()`（`plane_target_marker.py` 就踩过：夹爪歪 53°，见 WORKLOG 2026-09-29(二)）。
    **光看代码"读起来对"不算验证过**——要真的调一次函数看输出，或在启动时打一条自检日志
    （该节点现在启动就打印命令姿态与"离竖直向下多少度"，配错会直接告警）。
19. **`InteractiveMarkerServer` 会静默丢弃"外来"的拖动反馈**：`processFeedback()` 里有
    `if last_client_id != feedback.client_id and (now - last_feedback) < 1.0: return`——
    1 秒内有别的 client（RViz）动过同一个 marker，外部构造的 `InteractiveMarkerFeedback`
    会被**静默丢掉且不报错**。想离线测拖动链路必须先确认没有别的 client 在拖（很容易把
    用户自己的操作误判成代码 bug）。

## 8. Agent 工作提示

- 需要了解项目进展、历史决策或准备答辩材料时，先看根目录 `WORKLOG.md`（逐日日志，最新在上）与 `src/tasks/doc/decisions.md`。
- 修改前先读相关 `package.xml`、launch、YAML 与源码，确认实际调用链。
- 优先使用 `grep`/`glob` 定位；路径用相对工作空间根的形式书写。
- 需要构建/运行验证时，遵循第 5 节；不要假设 `catkin build`。
- 涉及硬件动作时，只给命令与说明，不代替用户执行真实硬件操作。
- 改完 `joint_slider_panel.cpp`（控制台）后**必须重启 launch 才生效**（Qt 二进制不在热重载范围）；
  离屏验证用 `QT_QPA_PLATFORM=offscreen rosrun agx_xarm_bringup joint_slider_gui __name:=joint_slider_test`
  （改名避免和用户正在用的窗口抢同一个节点名）。
- **`pkill -f plane_target_marker` 这类宽模式会误杀用户正在跑的节点**（脚本路径里含包名，
  `rosrun` 展开后就是 `devel/lib/agx_plane_target/plane_target_marker.py`）。要停测试节点请用
  `pkill -f "plane_target_marker.py _enable_arm"` 这种**带参数**的模式，或先 `rosnode list` 确认。
  误杀后恢复：`nohup rosrun agx_plane_target plane_target_marker.py &`（它自己不驱动硬件，
  但 RViz 里的红点会消失，控制台「平面红点」整行会置灰直到它回来）。
- **别把"服务立刻返回"当成"动作已完成"**：`plane_target/*` 都是立即返回、动作在节点自己的线程里跑。
  异步动作的进度必须靠话题回传（夹取用 `plane_target/grasp_status`，放入用 `plane_target/put_status`），
  不能靠服务响应推断。
