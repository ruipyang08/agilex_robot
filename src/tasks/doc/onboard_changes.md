# 解压后的修复与修改（给远程 AI）

基线是源码包 `agilex_ws-source-ee90218.tar.gz`（`DEPLOYMENT_REVISION` 为 `ee90218`）。下面写的是这台机器人在解压该包之后、相对基线做过的全部修复和功能修改的**最终状态**。

远程电脑上的 AI：从第 3 节起按顺序改源码，改到第 2 节描述的最终行为为止。不要用本机整个 `agilex_ws` 覆盖远程。本机目录里有 `build/`、`devel/` 和现场运行状态，那些不要拷。路径都相对仓库根目录。

中间过程里出现过、后来又被改掉的做法列在第 6 节。不要按第 6 节实现。

## 1. 不要改

- 六个点的坐标数值（x、y、z）不要改，只允许改点号和名字的对应关系（见 3.6）。坐标仍是 `measurements.md` M6 里那六个实测点，坐标系 `link_base_lift0`。
- 不要改 `link_base_lift0`、`deck_top` 的静态 TF。
- 不要改黄色安全区。它的唯一真源是 `agilex_ws/src/start/agx_xarm_bringup/scripts/safety_plane_viz.py` 的 `PLANE`。底面仍是 `deck_top + 0.028 m`，顶面 `deck_top + 1.298 m`。
- 不要改 SRDF 里名为 `home` 的位姿（六个关节都是 0）。要改的是名为 `initial` 的位姿，数值见 3.5。
- 不要删 `joint_slider_gui` 的源码。只是不再由 `setup_arm.launch` 启动它。
- 不要改旧的红点夹取服务 `/plane_target/grasp` 的行为。新抓取是另一个服务 `/plane_target/pick`。
- 夹爪刻度沿用 DH AG95：`0` 合拢到底，`1000` 全开。

## 2. 改完必须是这样

夹爪：`GRASP_OPEN = 1000`，`PUT_GRIP_CLOSE = 200`。力度和速度每次重读 `~grasp_force`、`~grasp_speed`。

安全高度：`PUT_APPROACH_RISE = 0.150` 米（150 mm）。放入和抓取的下降、抬高共用这一个数。

放入 `plane_target/put`（点号 1～6，进度话题 `/plane_target/put_status`）：

1. `enable_arm` 为真时，手臂离开当前位置之前先闭合到 200。闭合失败或这一步被急停，手臂不动。
2. 自由规划到该点上方 150 mm。
3. 直线下降到该点。
4. 张开到 1000。
5. 直线抬高 150 mm。
6. 自由规划回 SRDF 命名位姿 `initial`。

抓取 `plane_target/pick`（同一批点，进度话题 `/plane_target/pick_status`，服务类型 `agx_plane_target/PickPoint`）：

1. `enable_arm` 为真时，手臂不动之前先张开到 1000。停稳读数必须 ≥ `PICK_OPEN_MIN = 980`。不到这个数就视为还夹着东西或没张开，手臂不动。
2. 自由规划到该点上方 150 mm。
3. 直线下降到该点。
4. 闭合到 200。停稳读数若仍 ≥ `PICK_STILL_OPEN = 900`（现场出现过停在 941 仍被当成闭合成功），停在目标点，不抬升。
5. 直线抬高 150 mm。
6. 自由规划回 `initial`。

`enable_arm` 为假（干跑）时，放入和抓取都只规划「到上方点」，不下发任何夹爪指令，也不做直线下降。

竖直两段用 `compute_cartesian_path`，不套黄色安全区约束，碰撞检查保留，并按 `~vel_scaling` 重定时后再执行。到上方点和回 `initial` 仍带安全区约束，并把约束盒底面抬到目标点高度。

点号同时驱动 RViz 红球编号、`put N` 和 `pick N`。当前点号见 3.6，已经不是 M6 表格从上到下的旧顺序。

急停仍是 `rosservice call /plane_target/stop`，或 `put stop` / `pick stop`。`plane_target/home` 仍回到 `initial`。

## 3. 按这个顺序改

每一步都相对 `ee90218` 的源码。后一步不要把前一步的最终值改回去。

### 3.1 界面：只要六个红球

文件：

- `agilex_ws/src/start/agx_xarm_bringup/launch/setup_arm.launch`
- `agilex_ws/src/start/agx_xarm_bringup/rviz/arm_display.rviz`
- `agilex_ws/src/tasks/agx_plane_target/scripts/plane_target_marker.py`

改成：

- `setup_arm.launch` 删除节点 `joint_slider_gui`，不要再给它写启动参数。
- `arm_display.rviz` 删除显示项 `Plane Target`（`rviz/InteractiveMarkers`，话题 `/plane_target/update`）。保留 `Put Points`（话题 `/plane_target/put_points`，命名空间 `put_points`）。
- `plane_target_marker.py` 启动时不要再 `insert` 名为 `target` 的交互红点。`RETIRED_MARKERS = ("stop", "target")`。启动后用 `InteractiveMarkerUpdate.erases` 重复擦除这两个名字。`server.erase()` 只对本进程插入过的标记有效。
- `_set_dot_color()` 直接返回，运动过程中不要把红点画回来。
- `_publish_put_markers()` 里球和编号的 `header.stamp` 用 `rospy.Time()`（零时间戳）。不要用 `rospy.Time.now()`。晚开的 RViz 才能画出红球。
- `setup_arm.launch` 的 `arm_rviz` 节点加上：

```xml
<env name="DISABLE_ROS1_EOL_WARNINGS" value="1" />
```

否则 ROS Noetic 停服提示会挡住红球。

### 3.2 放入：出发前闭合到 200，安全高度 150 mm

文件：

- `agilex_ws/src/tasks/agx_plane_target/scripts/plane_target_marker.py`
- `agilex_ws/src/tasks/agx_plane_target/scripts/put`
- `agilex_ws/src/tasks/tests/test_put.py`
- `agilex_ws/src/start/agx_xarm_bringup/launch/setup_arm.launch`（注释里的高度）

改成：

- 常量 `PUT_GRIP_CLOSE = 200`，`PUT_APPROACH_RISE = 0.150`。单位是米。
- 放入顺序按第 2 节。文件头、干跑状态句、launch 注释里写死的「100 mm」全部改成 150 mm。
- `test_put.py` 里所有期望接近高度的 `0.100` 改成 `0.150`。顺序断言是：先 `gripper(200)`，再运动，到位后再 `gripper(1000)`。
- `scripts/put` 开头说明写成同一条顺序。

### 3.3 新增抓取服务

文件：

- `agilex_ws/src/tasks/agx_plane_target/srv/PickPoint.srv`（新建，字段与 `PutPoint.srv` 相同：请求 `int32 point`，响应 `bool success`、`string message`）
- `agilex_ws/src/tasks/agx_plane_target/CMakeLists.txt`（`add_service_files` 加入 `PickPoint.srv`；`catkin_install_python` 加入 `scripts/pick`）
- `agilex_ws/src/tasks/agx_plane_target/package.xml`（描述里写上 pick）
- `agilex_ws/src/tasks/agx_plane_target/scripts/plane_target_marker.py`
- `agilex_ws/src/tasks/agx_plane_target/scripts/pick`（新建，用法对齐 `scripts/put`：`pick <1～6>`、`pick status`、`pick stop`、`pick install`）
- `agilex_ws/src/tasks/tests/test_pick.py`（新建）
- `agilex_ws/src/tasks/tests/test_put.py`
- `agilex_ws/src/tasks/tests/test_grasp.py`
- `agilex_ws/src/tasks/tests/run_all.sh`
- `agilex_ws/src/start/agx_xarm_bringup/launch/setup_arm.launch`（注释里写上 pick 顺序）

改成：

- 服务名 `plane_target/pick`，类型 `PickPoint`。进度只发 `/plane_target/pick_status`，格式 `"<state>|<text>"`，`state` 为 `busy`、`ok`、`fail`。不要复用 `grasp_status` 或 `put_status`。
- 常量 `PICK_OPEN_MIN = 980`，`PICK_STILL_OPEN = 900`。顺序按第 2 节。
- `test_put.py` 和 `test_grasp.py` 在 `agx_plane_target.srv` 还没有 `PickPoint` 时，要往该模块上补一个同字段的替身，否则加载 `plane_target_marker.py` 会 `ImportError`。工作区生成过消息之后用真的 `PickPoint`。
- `run_all.sh` 的测试列表加上 `test_pick.py`。
- 改完在工作区执行 `catkin_make --pkg agx_plane_target`，再 `source devel/setup.bash`。不生成消息的话，节点启动时 `import PickPoint` 会失败。

### 3.4 初始位姿

文件：

- `agilex_ws/src/moveit_config/ranger_xarm_lifting_moveit_config/config/ranger_with_xarm6.srdf`

只改 `group_state name="initial" group="xarm6"` 的六个关节，单位弧度：

```xml
<group_state name="initial" group="xarm6">
    <joint name="joint1" value="0.034574"/>
    <joint name="joint2" value="-0.713773"/>
    <joint name="joint3" value="-0.582719"/>
    <joint name="joint4" value="0.000007"/>
    <joint name="joint5" value="1.296480"/>
    <joint name="joint6" value="-0.010319"/>
</group_state>
```

基线是 joint1～4 和 joint6 为 0，joint5 为 `-1.5708`。不要改 `home`。`rospack find ranger_xarm_lifting_moveit_config` 应指向这份源码；MoveIt 只在启动时读它。

### 3.5 六个点的当前点号

坐标不要改。把 `plane_target_marker.py` 的 `PUT_POINTS` 改成下面这张表。左边的数字是 `put` / `pick` / 红球编号。

```python
PUT_POINTS = {
    1: ("绿轴 -75 mm", 0.327302, 0.082902, -0.078002),
    2: ("中心", 0.329030, 0.007922, -0.078002),
    3: ("绿轴 +75 mm", 0.330758, -0.067058, -0.078002),
    4: ("绿轴 -75 mm，红轴 -75 mm", 0.252322, 0.081174, -0.078003),
    5: ("中心，红轴 -75 mm", 0.254050, 0.006194, -0.078003),
    6: ("绿轴 +75 mm，红轴 -75 mm", 0.255778, -0.068786, -0.078003),
}
```

同步两处只用于终端打印的 `NAMES`，否则命令行名字和实际运动对不上：

- `agilex_ws/src/tasks/agx_plane_target/scripts/put`
- `agilex_ws/src/tasks/agx_plane_target/scripts/pick`

`test_put.py` 里的 `_M6` 必须改成同一张点号表。基线测试仍按 M6 从上到下把 1 当成「中心」，不改的话 `test_put.py` 会失败。`measurements.md` M6 仍是实测坐标记录，点号以 `PUT_POINTS` 为准。

### 3.6 修复：升降节点脚本不可执行

`ee90218` 包在 Windows 上用 `tar.add` 打包，文件被存成 `0o666`。`lifting_ctrl_service_node.py` 因此是 644，`roslaunch` 报 not executable，`/lifter_1/LiftingMotorService` 不存在。

文件（只改权限，不改内容）：

- `agilex_ws/src/start/lifting_ws/src/lifting_ctrl/scripts/lifting_ctrl_service_node.py`

执行 `chmod 755` 该文件。被 launch 直接启动的其他 `*.py` 也要有用户可执行位。远程打包器若在 `packages/rover_core/src/rover_arm/deployment.py`，权限过滤必须包含 `*.py`，不能只包含 `*.sh`。

### 3.7 修复：升降探测串口改掉夹爪波特率

`JudgeCurrentMotorPort()` 找不到设备时会遍历 `/dev/ttyUSB*`。打开串口会按升降自己的 38400 改 termios，把夹爪那颗 FTDI 也改掉。夹爪驱动按 115200 收发，于是 `/gripper/states` 一直 `is_initialized=False`。

本机采用的修法是写死端口，不再遍历。只改这一份正在使用的配置：

- `agilex_ws/src/start/lifting_ws/src/lifting_ctrl/scripts/config/1_lifting_motor_config.json`

在 JSON 里增加（或改成）：

```json
"portName": "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
```

`lifting_motor_ctrl.py` 读到 `portName` 就不再调用 `JudgeCurrentMotorPort()`。不要改 `backup/` 里的配置。远程机器的 by-id 路径可能不同；原则是升降只用自己的串口，不要打开夹爪的 `/dev/DH_hand` 或夹爪所在的 `ttyUSB`。

现场临时恢复、不改代码：`stty -F /dev/ttyUSB2 115200`（夹爪实际设备号以现场为准）。

### 3.8 修复：视觉抓试管把正常升降状态当成故障

`tube_cap_detector_node.py` 的 `_lift_faulted()` 原来把 `externalCtrlMode` 和 `stopFlag` 当成故障。

- `externalCtrlMode` 只表示「由外部指令控制」。`lift` 命令正常工作时它一直为真，检测会永远返回 status=4。
- `stopFlag` 在升降停住时也为真（2026-10-22 实测：停在 0 mm 时 `stopFlag` 为真），同样会让 `/detect_tube_cap` 一直是 status=4。

文件：

- `agilex_ws/src/sensors/camera/agx_tube_perception/scripts/tube_cap_detector_node.py`

`_lift_faulted()` 只保留这些真故障：`overflowI`、`overflowV`、`encodeErr`、`posBiasOver`、`underVol`、`overLoad`、`fpsError != 0`。不要再 `or message.externalCtrlMode`，也不要再 `or message.stopFlag`。

检测还要求升降停在 100 mm（台面配置 `table_grid_3x3_v1`：台面 0.985 m 减 `arm_base_height_at_zero_lift_above_floor_m` 0.885 m）。升降在 0 时会报目标高度不匹配。这不是代码错误，调用前执行 `lift 100`。

## 4. 改完怎么验收

先 source：

```bash
source /opt/ros/noetic/setup.bash
source ~/ranger_arm_deploy/agilex_ws/devel/setup.bash
```

离线测试（不碰真机、不需要 roscore）：

```bash
python3 agilex_ws/src/tasks/tests/test_put.py
python3 agilex_ws/src/tasks/tests/test_pick.py
python3 agilex_ws/src/tasks/tests/test_grasp.py
```

或 `bash agilex_ws/src/tasks/tests/run_all.sh`。三条与本次改动直接相关的测试必须通过。

真机：

```bash
roslaunch agx_bringup hardware.launch
```

另一个终端：

```bash
rosrun agx_plane_target put 1
rosrun agx_plane_target pick 1
```

点号 1 到 6。停止：`rosrun agx_plane_target put stop` 或 `pick stop`。只回初始、不放入不抓取：`rosservice call /plane_target/home "{}"`。

同时确认：

- `lift status` 能连上 `/lifter_1/LiftingMotorService`。
- `/gripper/states` 的 `is_initialized` 为真，张开时 `position` 为 1000。
- 重启之后，回 `initial` 的关节角是 3.4 节那一组，不是 joint5 = −90°。

Python 节点和 launch 都是启动时读源码。改 `plane_target_marker.py` 后重启该节点。改 SRDF 后必须整段重启 `hardware.launch`，已经在跑的 `move_group` 不会换新的 `initial`。

## 5. 节点与命令对照

| 作用 | 名字 |
|---|---|
| 放入 | 服务 `/plane_target/put`，命令 `rosrun agx_plane_target put <1～6>`，进度 `/plane_target/put_status` |
| 抓取 | 服务 `/plane_target/pick`，命令 `rosrun agx_plane_target pick <1～6>`，进度 `/plane_target/pick_status` |
| 旧红点夹取 | 服务 `/plane_target/grasp`，不要改它的流程 |
| 回初始 | 服务 `/plane_target/home`，SRDF 名 `initial` |
| 急停 | 服务 `/plane_target/stop` |
| 六个红球 | 话题 `/plane_target/put_points`，坐标系 `link_base_lift0` |

## 6. 不要实现的中间状态

这些是现场改过、又被后面的步骤取代的描述。远程如果照做，结果会和本机不一致。

- 安全高度曾经是 100 mm。最终是 150 mm（`PUT_APPROACH_RISE = 0.150`）。
- 抓取曾经「下降之前不发夹爪指令」。最终是下降之前必须先确认张开到 1000，且读数 ≥ 980。
- 「闭合后仍停在 980 以上才不抬升」是错的。不抬升的阈值是 `PICK_STILL_OPEN = 900`。941 必须被挡住。
- 点号曾经等于 M6 表格从上到下（1 = 中心）。最终点号是 3.5 节那张表（1 = 绿轴 −75 mm）。
- 视觉故障判断曾经只去掉 `externalCtrlMode`、仍把 `stopFlag` 当故障。最终两者都不是故障。
