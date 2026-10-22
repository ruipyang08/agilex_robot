# 本机修改记录

基线是远程电脑打出的源码包 `agilex_ws-source-ee90218.tar.gz`（`DEPLOYMENT_REVISION` 为 `ee90218`）。远程工作区仍是这一版。下面每一条都是在这台机器人上、相对该基线做过的修改。

这份文件放在 `agilex_ws/src/tasks/doc/onboard_changes.md`，两边路径相同。

## 怎么用

远程电脑：从第一条起，按「改哪些文件」和「要改成什么样」改源码。不要用本机的整个 `agilex_ws` 覆盖远程，本机目录里还有 `build/`、`devel/` 和现场运行状态。改完在远程跑 `python3 agilex_ws/src/tasks/tests/test_put.py` 和 `python3 agilex_ws/src/tasks/tests/test_pick.py`，再按原流程重新打包。

本机：代码已经按这些条目改过。以后再改本机，在文末按「追加模板」加一条，日期用当天，编号顺延。把本文件拷回远程时，远程只补它还没有的条目。

不要改的东西：六个放入点坐标（`PUT_POINTS`，对应 `measurements.md` M6）、`link_base_lift0`、安全区、`plane_target/home` 回到的 SRDF 位姿 `initial`。`joint_slider_gui` 的源码留着，只是不再启动。

夹爪位置沿用 DH AG95：`0` 合拢到底，`1000` 全开。

## 2026-10-14　去掉控制台和可拖动红点

现场看到的仍是上一版：单独的「控制台」窗口，加上夹爪下那个可拖动红点（外面还有半透明大手柄）。六个编号红球被挡在一起。这一版只要六个红球，不要控制台，也不要那个红点。

改这些文件：

- `agilex_ws/src/start/agx_xarm_bringup/launch/setup_arm.launch`
- `agilex_ws/src/start/agx_xarm_bringup/rviz/arm_display.rviz`
- `agilex_ws/src/tasks/agx_plane_target/scripts/plane_target_marker.py`

要改成这样：

- `setup_arm.launch` 删掉节点 `joint_slider_gui`。不要再为它写启动参数。
- `arm_display.rviz` 删掉显示项 `Plane Target`（`rviz/InteractiveMarkers`，话题 `/plane_target/update`）。保留 `Put Points`（话题 `/plane_target/put_points`，命名空间 `put_points`）。
- `plane_target_marker.py` 启动时不要再 `insert` 名为 `target` 的交互红点。`RETIRED_MARKERS` 改为 `("stop", "target")`，启动后用 `InteractiveMarkerUpdate.erases` 重复擦除这两个名字，让已经开着的 RViz 丢掉旧红点。`server.erase()` 只对当前进程插入过的标记有效，擦除必须走 `server.publish()`。
- `_set_dot_color()` 改为直接返回，运动过程中不要把红点画回来。
- 急停仍是 `rosservice call /plane_target/stop`。`plane_target/put`、`/grasp`、`/home` 保留。

## 2026-10-14　六个红球用当前坐标，避免晚开的 RViz 不画

红球话题 `/plane_target/put_points` 上一直有 6 个球和 6 个编号，但标记时间戳停在节点启动那一刻。RViz 若在这之后才打开，用过期时间查 TF，查不到就不画。

改 `plane_target_marker.py` 里 `_publish_put_markers()`：球和编号的 `header.stamp` 用 `rospy.Time()`（零时间戳），让 RViz 用最新坐标。不要用 `rospy.Time.now()`。

另外，`setup_arm.launch` 里 `arm_rviz` 节点加上：

```xml
<env name="DISABLE_ROS1_EOL_WARNINGS" value="1" />
```

否则 ROS Noetic 停服提示会挡住甲板上的红球。

节点是启动时读源码的。改完必须重启 `plane_target_marker`，已经在跑的进程不会自己换新。

## 2026-10-15　放入前先把夹爪闭合到 200

基线的放入不闭合夹爪，只在到位后张开到 1000，所以全程是张开的。要求改成：到达红球之前闭合到 200，到位后松开，再回初始位置。

改这些文件：

- `agilex_ws/src/tasks/agx_plane_target/scripts/plane_target_marker.py`
- `agilex_ws/src/tasks/agx_plane_target/scripts/put`
- `agilex_ws/src/tasks/tests/test_put.py`

要改成这样：

- 增加常量 `PUT_GRIP_CLOSE = 200`。松开仍是 `GRASP_OPEN = 1000`。力度和速度继续用每次重读的 `~grasp_force`、`~grasp_speed`。
- `plane_target/put` 在 `enable_arm` 为真时，手臂离开当前位置之前先闭合到 200。闭合失败或这一步被急停，手臂不动。
- 其后顺序不变：上方 100 mm → 直线下降到红球 → 松开到 1000 → 直线抬高 100 mm → 回到 SRDF `initial`。抬高留着，避免松开后直接回家时扫过刚放下的物体。
- `enable_arm` 为假（干跑）时仍只规划「到上方点」，不下发任何夹爪指令。
- `scripts/put` 开头的说明改成这条顺序。`test_put.py` 的顺序、失败路径和急停断言改成「先 gripper(200)，再运动，到位后再 gripper(1000)」。

真机验证仍用：

```bash
source /opt/ros/noetic/setup.bash
source ~/ranger_arm_deploy/agilex_ws/devel/setup.bash
roslaunch agx_bringup hardware.launch
```

另一个终端：

```bash
rosrun agx_plane_target put 1
```

点号 1 到 6。中途停止：`rosrun agx_plane_target put stop`。回初始位置、不放入：`rosservice call /plane_target/home "{}"`。

## 2026-10-16　放入和抓取的安全高度改为 150 mm

上一条写入时，上方接近高度还是 100 mm。现场改成 150 mm。下降和抬高共用这一个数，六个放入点坐标不动。黄盒子底面仍是 `deck_top + 0.028 m`，不要改。

改这些文件：

- `agilex_ws/src/tasks/agx_plane_target/scripts/plane_target_marker.py`
- `agilex_ws/src/tasks/tests/test_put.py`
- `agilex_ws/src/start/agx_xarm_bringup/launch/setup_arm.launch`

要改成这样：

- `PUT_APPROACH_RISE = 0.150`（米）。文件头、干跑状态句和 `setup_arm.launch` 里写死的「上方 100 mm」一并改成 150 mm。
- `test_put.py` 里期望的 `0.100` 改成 `0.150`。
- 改完重启 `plane_target_marker`。远程验证：`python3 agilex_ws/src/tasks/tests/test_put.py`。

## 2026-10-16　抓取与放入平行，出发前先确认夹爪张开

放入是出发前先闭合到 200。抓取用同一批六个点和同一个 `PUT_APPROACH_RISE`（150 mm），但夹爪时机相反。点 1 抓完回到初始位置时，闭合被报成成功，读数却停在 941。接着抓点 2，在目标点闭合后再抬升，机械臂报错 31，抬升被取消。所以抓取不能从还闭合着的夹爪出发，也不能在夹爪仍停在张开区间时抬升。

改这些文件：

- `agilex_ws/src/tasks/agx_plane_target/scripts/plane_target_marker.py`
- `agilex_ws/src/tasks/agx_plane_target/scripts/pick`
- `agilex_ws/src/tasks/agx_plane_target/srv/PickPoint.srv`
- `agilex_ws/src/tasks/agx_plane_target/CMakeLists.txt`
- `agilex_ws/src/tasks/agx_plane_target/package.xml`
- `agilex_ws/src/tasks/tests/test_pick.py`
- `agilex_ws/src/tasks/tests/test_put.py`
- `agilex_ws/src/tasks/tests/test_grasp.py`
- `agilex_ws/src/tasks/tests/run_all.sh`
- `agilex_ws/src/start/agx_xarm_bringup/launch/setup_arm.launch`

要改成这样：

- 新服务 `plane_target/pick`（`PickPoint.srv`，点号 1~6，和放入同一批点）。`CMakeLists.txt` 加入这个 srv 和 `scripts/pick`。进度在 `/plane_target/pick_status`，不占用夹取或放入的状态话题。旧的红点夹取 `/plane_target/grasp` 不动。
- `enable_arm` 为真时的顺序：先张开到 1000，停稳读数要达到 `PICK_OPEN_MIN = 980`，否则视为还夹着东西，手臂不动；再到上方 150 mm；直线下降；闭合到 200；若停稳读数仍 ≥ `PICK_STILL_OPEN = 900`（941 就落在这里），停在目标点，不抬升；否则直线抬高 150 mm，再回 SRDF `initial`。
- `enable_arm` 为假时只规划到上方点，不下发任何夹爪指令。
- 终端是 `rosrun agx_plane_target pick 1` … `pick 6`，停止是 `pick stop`。`test_put.py`、`test_grasp.py` 在服务定义还没生成时要能退回 `PickPoint` 替身。`run_all.sh` 加上 `test_pick.py`。
- 改完在本包目录的工作区跑 `catkin_make --pkg agx_plane_target`，重新 source `devel/setup.bash`，再重启 `plane_target_marker`。远程验证：`python3 agilex_ws/src/tasks/tests/test_pick.py`。

## 2026-10-16　初始位置改成当前示教姿态

`plane_target/home`、放入和抓取的最后一步都回到 SRDF 命名位姿 `initial`。原来是 joint1–4、joint6 为 0，joint5 为 −90°。现场把末端调成竖直向下后，用当时的关节角替换这一组。`home`（六个关节都是 0）不动。

改这个文件：

- `agilex_ws/src/moveit_config/ranger_xarm_lifting_moveit_config/config/ranger_with_xarm6.srdf`

要改成这样：

- `initial` 的关节角（弧度）：joint1 `0.034574`，joint2 `-0.713773`，joint3 `-0.582719`，joint4 `0.000007`，joint5 `1.296480`，joint6 `-0.010319`。
- MoveIt 只在启动时读这份 SRDF。改完必须重启 `roslaunch agx_bringup hardware.launch`，已经在跑的 `move_group` 仍会回到旧的初始位置。

## 2026-10-17　打包丢可执行位：升降节点起不来

整机 `hardware.launch` 起来后 `/lifter_1` 不在节点列表，`lift status` 报找不到 `/lifter_1/LiftingMotorService`。原因是 `lifting_ctrl_service_node.py` 权限是 644，roslaunch 起不来（rosrun 提示 "not executable"）。`ee90218` 包里这个文件也是 644——打包器在 Windows 上用 `tar.add`，所有文件被存成 `0o666`；10-14 那次只给 `*.sh` 和 `put` 补了 0755，`.py` 漏了。

改这些文件：

- `agilex_ws/src/start/lifting_ws/src/lifting_ctrl/scripts/lifting_ctrl_service_node.py`（只改权限位）

要改成这样：

- `chmod 755 lifting_ctrl_service_node.py`，本机已验证：单独起 `roslaunch lifting_ctrl start_motor.launch` 后 `lift status` 正常（0 mm、已初始化、下限位）。
- 远程打包器（`packages/rover_core/src/rover_arm/deployment.py`）的权限过滤器要覆盖 `*.py`，不能只覆盖 `*.sh`；否则每次 Windows 出包都会复现。
- 全量检查同类问题：`find agilex_ws/src -name '*.py' -perm -u+x | wc -l` 与 `find agilex_ws/src -name '*.py' ! -perm -u+x | wc -l` 对比，被 launch 直接调用的节点脚本必须有 +x。

## 2026-10-17　升降启动探测串口会把夹爪的波特率改掉

整机 `hardware.launch` 起来后 `/gripper/states` 一直 `is_initialized=False`、`position=0`，发 `initialize:true` 和 `position:1000` 都没反应，但夹爪指示灯是亮的。停掉 launch 后用 pyserial 直接读 `/dev/ttyUSB2`（Modbus 115200），读回 `init=1 / position=1000`，说明夹爪本体正常。

根因：`lifting_ctrl` 的 `JudgeCurrentMotorPort()` 在找不到设备时会遍历 `/dev/ttyUSB*`，逐个打开探测；打开时会按自己的波特率配置 termios，而 termios 是设备级的——它把夹爪那颗 FTDI（ttyUSB2）也改成了 38400，`dh_gripper_driver` 之后按 115200 收发自然是死的。升降节点起不来的那段时间（脚本缺 +x）夹爪一切正常，`chmod` 修好、升降一起来之后夹爪就"坏"，正是这个顺序。

改这些文件（本机已于 2026-10-17 按下面的首选方案修好）：

- `agilex_ws/src/start/lifting_ws/src/lifting_ctrl/scripts/lifting_motor_ctrl.py`（`JudgeCurrentMotorPort`）或 `scripts/config/<id>_lifting_motor_config.json`

要改成这样：

- 首选：在 `config/<motor_id>_lifting_motor_config.json` 里写死 `portName`（值为 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`），代码读到 `portName` 就不再遍历 `/dev/ttyUSB*`，夹爪的 termios 不会被碰。
- 次选：`JudgeCurrentMotorPort` 探测时跳过 `/dev/DH_hand` 指向的端口，或探测结束后恢复原 termios。
- 临时恢复（不改代码）：`stty -F /dev/ttyUSB2 115200`，驱动下一轮轮询就能读回数据，实测有效。

本机若已重启 launch：先 `stty` 恢复再跑 `pick`/`put`。远程验证：重启 launch 后同时确认 `lift status` 正常 **且** `/gripper/states` 的 `is_initialized=True`、`position=1000`。

## 2026-10-17　视觉抓试管链路在真机上的三个前置条件

用 `hardware.launch enable_tube_grasp:=true` 或单独 `roslaunch agx_bringup tube_grasp.launch` 起视觉链后，`/detect_tube_cap` 连续返回 status=4（STALE_FRAME），三种 detail 分别指向三个前置条件：

1. `lift feedback reports a fault` —— `tube_cap_detector_node.py` 的 `_lift_faulted()` 把 `externalCtrlMode` 当故障。该位是 `stateBit & 0x80`，表示"由外部指令控制"，`lift` CLI 正常操作时恒为 True，永远不会清。已在真机去掉这一项，保留 overLoad / underVol / encodeErr / posBiasOver / overflow / stopFlag / fpsError 这些真故障位。
2. `lift feedback target does not match the request` —— 检测要求升降**停在 100 mm**：`table_grid_3x3_v1` 的台面 0.985 m 减去配置里 `arm_base_height_at_zero_lift_above_floor_m: 0.885` 正好 0.100 m。升降在 0 时必然不匹配，先 `lift 100` 再检测。
3. `stopFlag` 是升降在限位停住时锁存的软件急停位，做过一次真实动作（例如 `lift 100`）后会自己清掉，不需要改代码。

改这些文件：

- `agilex_ws/src/sensors/camera/agx_tube_perception/scripts/tube_cap_detector_node.py`（`_lift_faulted` 里的 `or message.externalCtrlMode` 改为 `or False` 并注明原因）

本机验证：升降 100 mm 时调用 `rosservice call /detect_tube_cap "{fixture_profile_id: 'table_grid_3x3_v1', target_cap_color: 'green', mode: 1}"`，返回 status=2 `no valid green target cap`（画面里没有绿盖试管，绿色像素占比 0.000%），说明检测链路本身已跑通，不是门禁问题。远程验证：放一根绿盖试管后同一条命令应返回 status=0 并给出 `cap_top`。

## 追加模板

以后在本机改代码，复制下面这一段加到文末，编号用当天日期。远程还没有这条时，按它改远程源码。

```markdown
## YYYY-MM-DD　一句话标题

为什么改。

改这些文件：

- `相对仓库根目录的路径`

要改成这样：

- 行为，不要只写函数名。

本机若节点已在跑：要不要重启哪个节点。远程验证：哪条命令或哪个测试。
```
