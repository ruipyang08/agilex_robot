# 项目工作日志

> **用途**：逐日记录 AgileX Ranger Mini + 升降柱 + xArm6 移动操作机器人的项目进展，用于答辩时快速回溯「做了什么、为什么这么做、验证到什么程度」。
>
> **维护约定**
> - 每天在最上方新增一个 `## YYYY-MM-DD` 小节（最新在上），按「目标 / 改动 / 验证 / 结论 / 待办」组织
> - **详细数据不写在这里**：原始记录、决策理由、接口契约放 `src/tasks/doc/`（本日志只写摘要 + 指针，避免同一个数写两遍）
> - 关键结论要带**证据**（实测数字、命令、验证方法），答辩时可直接引用
>
> **新建条目模板**（复制到文件最上方分隔线之后）
>
> ```markdown
> ## YYYY-MM-DD —— 一句话标题
>
> ### 一、目标
> ### 二、完成内容
> ### 三、踩坑记录（避免重犯）
> ### 四、改动文件清单
> ### 五、待办 / 下一步
> ### 六、关键命令备忘
> ```
>
> **详细文档索引**
> | 文档 | 内容 |
> |---|---|
> | `src/tasks/doc/decisions.md` | 关键决策记录：为什么这么设计、放弃了什么方案 |
> | `src/tasks/doc/measurements.md` | 实测数据：升降标定 M1、TCP M2、示教 M3、夹爪状态 M4、碰撞区干跑 M5 |
> | `src/tasks/doc/interface.md` | 任务层接口契约（冻结版） |
> | `src/tasks/README.md` | 任务层总览与改动追踪表 |
> | `AGENTS.md` | 给 AI/开发者的工作空间导航与约定 |

---

## 2026-10-22(四) —— 真机首次抓取绿盖：红点链路单次抓取跑通（M7 落点 + M4 语义补齐）

### 一、目标

启动机器人，抓起试管架上的绿色管盖。原计划走 `tube_grasp` 自动链路，准入阶段即被挡住，改为先用已验证的 `plane_target` 红点链路做单次抓取。

### 二、完成内容

- **全自动链路准入实测失败，两条硬约束**：
  - `run_robot_workflow _config:=navigate_dock_grasp.yaml _check_only:=true` → `VALIDATE: unknown registered target: wb-100`。本机没有任何工位注册表（`registry_file` 默认空，全盘无 `*registry*.yaml`）
  - 只放抓取任务的配方 → `grasp requires a preceding verified navigation/hold task`，`grasp_test_tube` 不能单独派发
- **红点链路单次抓取成功**（落点与判据见 `src/tasks/doc/measurements.md` M7）：
  - 落点 deck_top **(0.259, −0.005, 0.100)**，`plane_z` 用默认 0.100
  - 夹爪：目标 0、力 50 → 停在 **268**、`grip_state=2`
  - 抬起 340 mm 回 `initial` 后 position 268→267，状态仍为 2 → 抓取牢固；随后原位张开 1000 放回
- 顺带把 **M4 的 `grip_state` 语义表**按实测补齐：1=空闲/张开、2=夹住、3=完全闭合

### 三、踩坑记录（避免重犯）

1. **第一次瞄偏 15 mm 就空夹**。瞄 deck_top (0.274, −0.005) 时闭合到 0、`grip_state=3`，肉眼看着"差不多"其实是偏前 1~2 cm。**判据只能是 `position` 和 `grip_state`，不能看爪子在不在盖上方**。
2. **官方检测器抓不到这支盖**：绿盖实测 HSV H=88，而 `test_tube_fixture_profiles.yaml` 的窗口是 [40,80]，严格阈值下绿色像素为 0。要跑自动链路必须先改窗口。
3. **`/detect_tube_cap` 被升降状态卡死**：升降静止时 `stopFlag=True`，检测服务直接返回 `lift feedback reports a fault`（status=4）。本次抓取升降一直在 0 mm，服务全程不可用。
4. **盖面反光，深度图在盖上全是 NaN**，不能靠 RealSense 测盖高；腕装相机在抓取位离指尖只有 11 cm，也拍不到有效画面。高度只能靠像素反投影 + 人眼校核。
5. **`roslaunch` 不能用 `( ... &)` 脱离终端**：工具 shell 退出时整组收到信号，27 个节点会在 30 s 内全部退出，日志里表现为一连串 `KeyboardInterrupt` / `init_node interrupted`。要用后台托管方式起。
6. **不要绕过 `plane_target` 自己发 MoveIt 位姿目标**：直连 `moveit_commander` 设的 z 和实际落点对不上（要 0.52 却停在 1.075），并报 `ABORTED: TIMED_OUT`；`plane_target` 的 `go`/`home` 才带安全区约束。

### 四、改动文件清单

- 无代码改动。本次只新增/填写文档：
  - `src/tasks/doc/measurements.md`：新增 **M7 绿盖抓取点**；填写 **M4 `grip_state` 语义表**
  - `WORKLOG.md`：本条

### 五、待办 / 下一步

- ⏸ `can0` 仍是 DOWN（`sudo ip link set can0 up type can bitrate 500000` 需要密码），底盘没动过
- ⏸ 全自动链路要真正跑起来，先得做工位注册：wb-100 的地图位姿、AR 码 100、相机/激光摘要；仓库里只有仿真占位样例
- ⏸ 检测器 HSV 窗口 [40,80] 与现场绿盖 H=88 不匹配，改窗口后 `/detect_tube_cap` 才可能返回 SUCCESS
- ⏸ 升降 `stopFlag=True` 被判成 fault 这条逻辑要复核：静止本来就该是 True

### 六、关键命令备忘

```bash
# 起硬件（不起底盘）+ 试管感知
roslaunch agx_bringup hardware.launch start_base:=false enable_tube_grasp:=true

# 红点抓取四步
rostopic pub -1 /plane_target/nudge geometry_msgs/Vector3 "{x: -0.0765, y: -0.005, z: 0}"
rosservice call /plane_target/go        # 到红点
rosparam set /plane_target_marker/grasp_position 0
rosservice call /plane_target/grasp     # 张开 → 到位 → 闭合
rosservice call /plane_target/home      # 抬起验证

# 看是否夹住
rostopic echo -n1 /gripper/states       # 268 / grip_state=2 = 夹住；0 / 3 = 空夹
```

---

## 2026-10-14(二) —— 放入点落地：`put <1~6>` 服务 + 六红球 Marker + 终端命令（M6 / D22）

### 一、目标

需求：把夹取到手的物体放到示教好的六个放入点上；接口暂定终端敲 `put1..6`；RViz 里六个点要有对应
红球（新发布），且**不能和原有控制台冲突**。计划稿：工作区根目录 `放入点与安全高度修改计划.md`。

### 二、改动摘要（理由与放弃方案见 `decisions.md` D22）

| 位置 | 改动 |
|---|---|
| `tasks/doc/measurements.md` | 新增 **M6**（六个放入点 + 上方接近点，frame `link_base_lift0`），按计划稿原文插入 |
| `tasks/agx_plane_target/srv/PutPoint.srv` | 新增服务定义（`int32 point` 1~6 -> `success`/`message`）；`CMakeLists.txt` / `package.xml` 补 `message_generation` / `message_runtime`（规则 A） |
| `tasks/agx_plane_target/scripts/plane_target_marker.py` | `plane_target/put` 五步（上方点 -> 直线下降 -> 松开 1000 -> 直线抬高 -> 回 `initial`）、`put_status` 话题、六红球 MarkerArray、黄区约束新增 `goal_point` / `floor` |
| `tasks/agx_plane_target/scripts/put` | 终端命令：`put <1~6>` / `putN` / `put status` / `put stop` / `put install`（自动补 ROS 环境，用法对齐 `~/bin/lift`） |
| `start/agx_xarm_bringup/rviz/arm_display.rviz` | 新增 `MarkerArray` 显示项 `/plane_target/put_points`（红球默认可见） |
| `tasks/tests/test_put.py` + `run_all.sh` | 新增离线测试并挂进 run_all（11 组、102 条断言） |
| `tasks/README.md` / `AGENTS.md` / `ONBOARD_CODE_MAP...` / `setup_arm.launch` 注释 | 接口、用法与索引同步；控制台 `joint_slider_panel.cpp` **未改**（按需求不与控制台耦合） |

### 三、关键设计（避免以后返工）

- 点号**随服务请求**传入：param + `Trigger` 会留下"点号已改、服务还没调"的窗口，且响应里看不出放了哪个点。
- 进度走**独立话题** `put_status`：控制台「夹取」行订阅的是 `grasp_status`，共用会让它把放入显示成夹取。
- 竖直两段 = `compute_cartesian_path` 直线（碰撞检查保留）且**清掉黄区路径约束**（示教点可能在盒外）；
  另两段保留黄区约束并把约束盒底面抬到放入点高度（"不降回放入高度"）。
- ⚠ MoveIt 的 Cartesian 服务**固定按全速**做时间参数化（Noetic capability 写死 `computeTimeStamps(rt, 1.0)`，
  忽略 `~vel_scaling`），所以下降前在客户端 `retime_trajectory(..., ~vel_scaling)` 重定时；失败退回原轨迹并 `logwarn`。
- 干跑只能验证第 1 步：不动机械臂就没法从"假设的上方点"继续规划直线段，所以第 2~5 步**明说跳过**（不假装）。

### 四、验证

| 项 | 结果 |
|---|---|
| `test_put.py`（开发机桩环境，无 ROS 工作区） | ✅ **102 条 ok / 0 FAIL**（exit 0） |
| `test_grasp.py` 回归（同一套桩） | ✅ 60 条 ok / 0 FAIL（与既有基线一致） |
| M6 数值对账（脚本比对计划稿 vs 代码常量） | ✅ 6 点全等；上方点 = 放入点 +0.100 m |
| M6 正文与计划稿逐行比对 | ✅ 逐字一致（只多一个章节分隔线） |
| `python -m py_compile`（节点 + CLI） | ✅ |
| `scripts/deploy_check.py` | ✅ FAIL=0（WARN 2，与既有基线相同） |
| 容器 clean `catkin_make`（hardware profile） | ⚠ 容器只有 7.7 GB 内存，`-j$(nproc)=20` 在 94% 处 OOM 杀掉一个 `cc1plus`；补跑 `catkin_make -j4` 增量 **EXIT=0**（环境限制，非代码问题） |
| 容器 `run_all.sh`（真机同款 Python 3.8 + 真消息） | ✅ **56 PASS + 162 ok / FAIL=0**，`==> 全部通过`（test_put 贡献 102 ok；旧记 116 为升降/夹取那批） |
| 假臂 headless 验收（`fake_plane_target.launch` + 新增 `agx_sim_fakearm/scripts/fake_ag95.py`） | ✅ `put 3` 五步全过：`put_status` 依次 1/5~5/5 后 `ok｜放入完成…停在 1000`；动作窗口内假控制器流出 **91 条**关节状态、收尾 joint5→-1.5707（= `initial`）；收尾 TF tcp = `[0.398, 0, 0.285]`；`put_points` 六球坐标与 M6 一致 |
| 真机端到端 | ⏸ 未做（需清场 + 物理急停 + 真实物体） |
| 打包产物核验（部署前） | 首次出包 `8d3238d` 核包发现缺 put 三件（打包早于实现完成）+ 全员 `0o666`；修 `deployment.py` 恢复 +x 后重出包并核验：`srv/PutPoint.srv` / `scripts/put`(rwxr-xr-x) / `tests/test_put.py` / `DEPLOYMENT_REVISION` 全对 |

### 四之二、踩坑

- **`test_put.py` 的桩在 Python 3.8 上炸**：`rospy_stub.Time.now = staticmethod(lambda: None)`
  在 3.10+ 可以调用（staticmethod 对象从 3.10 起可调用），容器/真机的 3.8 **不可调用**
  （`TypeError: 'staticmethod' object is not callable`）。开发机（3.14）跑绿、容器里第一次就暴露——
  改成普通 lambda 后两处都过。**教训：桩代码也受 Python 版本差异影响，必须两种环境都跑。**
- **Windows 打包丢可执行位**：`deployment.py` 在 Windows 上用 `tar.add` 出包，所有文件被存成
  `0o666`（没有 POSIX +x 位）。既有链路不受影响（rosrun 走 devel 拷贝、脚本用 `bash`/`python3`
  起），但新的 `scripts/put` CLI 需要在真机**直接执行**，解压后会 Permission denied。
  打包过滤器现在对 `*.sh` 和 `put` 恢复 `0o755`。另发现 18:02 打的 `4022cef` 包比 put 三件
  （srv/CLI/test_put）早约两小时、内容不含 put，已作废删除——**拷包前必须核对包里有
  `srv/PutPoint.srv`、`scripts/put`、`tests/test_put.py`**。

### 五、待办 / 下一步

1. ⏸ 真机：重建后先 `plane_target_execute:=false` 干跑，再实动（清场 + 物理急停）。
   **现场须知**：put 期间别按控制台「回到初始位置」（手臂会被 `busy` 拒绝，但按钮会先发一条夹爪
   张开指令把物体松开），也别拖夹爪滑条。
2. ⏸ 现场微调六点时的顺序：先改 M6 -> 再改 `plane_target_marker.py` 的 `PUT_POINTS` ->
   同步 `test_put.py` 的硬编码期望值（三者对账）。
3. ⏸ 环境：容器只有 7.7 GB 内存，clean `catkin_make -j$(nproc)=20` 会 OOM（本次在 94% 处杀了一个
   `cc1plus`）。要么给 Docker 提高内存上限，要么让构建脚本按内存选 job 数（本次是用 `-j4` 增量补跑过的）。
   下一次真正作为出包闸门的 clean 构建前先把这条处理掉，否则规则 F 会在同一处失败。

### 六、关键命令备忘

```bash
put 3                                          # 放入点 3（绿轴 -75 mm）
put install                                    # put / put1..put6 装到 ~/bin
put status                                     # 最近一次放进的进度/结果
put stop                                       # 等同 /plane_target/stop
rosservice call /plane_target/put "{point: 3}" # 原生服务用法
rostopic echo -n1 /plane_target/put_status
```

---

## 2026-10-14 —— 真机 feedback 回传：13e034d 包在 clean catkin_make 暴露的问题全部落回仓库

### 一、目标

真机把 13e034d 源码包编起来时卡了一批 clean 并行 `catkin_make` 问题，真机侧逐个修复并
通过反馈套件（patch / 契约 / 体检脚本 / 新部署文档）回传。本节把修复全部落回仓库，
并把"真机才暴露"的构建/launch 条件固化成 offboard 机械闸门，下次部署不再重演。

### 二、根因（为什么 offboard 没看出来）

已沉淀为契约 `docs/deployment/offboard_deployment_contract.md`（规则 A–H）。最重要一条：
offboard 从未做过 clean 并行 `catkin_make`（一直 Docker + `catkin build` + 增量），
包间消息生成顺序、package.xml 依赖完整性、launch 同名双节点这类问题被掩盖；
真机干净环境 `-j8` 一次性全部暴露。**构建口径就此拍板：catkin_make 为准。**

### 三、改动

1. 应用真机补丁（20 文件，`docs/deployment/onboard_feedback_2026-10-14/` 存档原件，
   应用目录勘误已随档注明）：
   - 规则 B：11 个 CMakeLists 补 `add_dependencies(${catkin_EXPORTED_TARGETS})`
     （`dh_gripper_driver`、`mark_listenner`、`agx_xarm_bringup`、imu×2、
     `pointcloud_to_laserscan`、`rf2o`、`rslidar_sdk`、`velodyne_laserscan`、
     `xarm_api`、`run_recorded_traj`）
   - 规则 A：`agx_xarm_smach` 补 `agx_pick_msg` 三类依赖
   - 规则 C：删 `agx_xarm_smach` 三个脚本里的 `/home/...` 死路径
   - 规则 D：`hardware.launch` 显式 `open_gripper:=false`（消除 `/dh_gripper_driver`
     同名双节点）+ 雷达 RViz 走 `show_rviz` 参数（hardware 传 false → 单 RViz）
   - 规则 E：rf2o `verbose:=false`；`mark_listenner` 的 `no data!` 降 DEBUG
2. `docs/deployment/onboard_copy_procedure.md` 换为 `catkin_make` 口径 + 体检流程版。
3. 新增 `scripts/deploy_check.py`（静态体检 + `--launch-ros` 顶层 launch 图检查）。
4. 打包器升级：出包前自动跑 `deploy_check`，FAIL 拒绝出包；规则 G 生成物
   （`.catkin_workspace` / `catkin_generated` / `catkin_make.cache`）进排除集。
5. `ros1_workspaces`：`hardware` profile 改为强制 clean `catkin_make -j$(nproc)`
   （`tool: make`），与真机构建条件对齐；simulation profile（Isaac 链）维持原状。
6. 口径同步：根 README Hardware 段改 `catkin_make`；`AGENTS.md` 构建节新增
   "打包前闸门"小节（规则速记 + 命令）。
7. 新增流程准绳 `docs/development/modification_guidelines.md`（功能修改准则 v1：
   修改前判断 / 规则索引 / 验证链 / 记录纪律 / 上真机 / 回程闭环 / 通过判据表 / 新功能落地清单），
   根 README 与 `AGENTS.md` 已挂指针。

### 四、验证

| 项 | 结果 |
|---|---|
| 补丁 `git apply --check` | ✅ 干净（20 文件无冲突） |
| `deploy_check.py` 静态体检 | ✅ FAIL 0 / WARN 2（rslidar_sdk 上游源码注释里的他人路径，契约允许的记录在案例外） |
| Docker 内 clean catkin_make + run_all.sh + launch 图检查 | ✅ 全过：`catkin_make -j8` EXIT=0（含 xarm_api，无需例外）；`run_all.sh` exit=0 全绿（实测 116 断言：56 PASS + 60 ok）；`deploy_check --launch-ros` 六入口 FAIL 0（hardware.launch 双夹爪节点与双 RViz 已被本次修复根治）；终检静态 FAIL 0 / WARN 2 |
| 重新打包（新 revision） | ✅ `tmp/deployment/agilex_ws-source-<HEAD>.tar.gz`——以包内 `DEPLOYMENT_REVISION` 为准（本日志写入提交本身就会改变 HEAD，故不写死哈希避免自指漂移） |

### 五、待办 / 下一步

1. 真机 `~/.bashrc` 的 source 路径修正（真机侧已改好，仓库无对应物，无需操作）。
2. 新包传真机复测时，真机侧同样从 clean 状态验证一次，增量不作为验收依据。

---

## 2026-10-13 —— 迁移：把真机验证工作区（agilex_ws_temp）的已验证功能合入本工作区

> 背景：`agilex_ws_temp` 是真机上跑通的 catkin 工作区快照（升降标定回归、平面红点 GO/夹取、
> 示教修复都在那里完成并真机回归过）；本工作区（agilex_ws）是 offboard 重构版，含更多新代码
> （tube 任务框架、导航栈、Docker/Isaac 仿真），但从未在真机验证。本次把 temp 中**真机验证过、
> 本工作区缺失**的语义逐项合入，保留本工作区新增框架，不做整目录覆盖。

### 一、目标

1. 让 `plane_target` 的 GO/夹取在本工作区可用（此前 canonical xacro 缺 `tcp_link`，任何
   MoveIt 目标都会报 `Link 'tcp_link' not found`）。
2. 让真机 bringup 的语义与 temp 的真机回归一致（夹爪驱动鲁棒性、xArm 控制器的拖动示教/
   急停恢复/模式自动切换、MoveIt 执行配置）。
3. 不破坏本工作区已通过的仿真链路（Isaac facade、假臂 bringup、tube 任务验收）。

### 二、逐项取舍（核心决策）

| 冲突 | 结论 | 依据 |
|---|---|---|
| xacro：temp 有 `tcp_link`(0.170 m)+真机标定注释；本工作区有升降视觉件/相机 link/保守碰撞原语 | **只把 tcp_link 增量注入**，保留本工作区其余模型结构 | tcp_link 是 plane_target 的硬依赖；升降视觉件/相机 link 被 Isaac 资产与 tube 验收引用（`usd_lift_mechanism.py` 等） |
| 升降关节名：temp=`lift_joint`(0..0.6)；本工作区=`lifting_joint`(0..0.7, 到升降台车的菱柱链) | **保留 `lifting_joint`**，把 `lift_joint_bridge.py` 改为 `~joint_name` 参数化（默认 `lifting_joint`） | Isaac/仿真侧 facade 硬编码 `lifting_joint`（`gateway.py`、`lift.py`）；URDF 上限不构成安全约束（真机行程由驱动 upLimitHeight=600 门禁把守），两侧语义以参数桥接 |
| `moveit.rviz` Fixed Frame：本工作区写 `camera_link`（模型中不存在！） | 采用 temp 的 `dummy_base_link` + 显示项数值 | camera_link 不存在于任一模型 —— 本工作区此文件照不出机器人 |
| `move_group.launch` manage_controllers true→false；controllers.yaml 加显式控制器名；trajectory_execution 容差 0.01→0.05；realMove_exec 联合关节源 | 全部采用 temp | 四项构成一组真机执行契约（控制器已由 xarm server 拉起；滑条/夹爪/升降关节源要进 MoveIt） |
| dh_gripper_driver（初始化鲁棒性、单次读寄存器、tcflush） | 整包 4 文件采用 temp | 真机 2026-09-29 验证 |
| scout_xarm/xarm_controller 的 xarm_hw.{h,cpp}（drag_teach、急停保持存活、0.25 rad 跳变拒绝、模式自动切换） | 采用 temp | 与 `admittance_teach.py`（两侧一致）的 `/xarm/drag_teach` 契约配套 |
| mark_listenner / ar_track_alvar / agx_pick_msg / agx_xarm_pick / ranger_messenger.cpp / ugv_sdk | **保留本工作区**（新鲜度守卫、publish_tf 参数、新消息、tube 框架、防 NaN 底盘修复、asio 适配都是本工作区更新的版本） | 逐 diff 审过方向 |
| agilexpro launch/param/map | 保留本工作区（hardware.launch 以 `start_processing` 参数调用 open_lidar.launch，temp 版无该参数会解析失败） | 参数耦合 |
| SRDF：temp 的 gripper 组（finger_joint + 开/合态 + passive 关节） | 增量合入 | fake_controllers 的 gripper 初始位姿与夹取规划需要它；碰撞对表保留本工作区（与保守碰撞原语配套） |

### 三、随带完成的配套改动

1. **假臂 overlay 退役**：`sim/ros1_ws/src/agx_sim_fakearm/urdf/ranger_xarm6_lifting_2431_fakearm.xacro`
   删除；`fake_plane_target.launch` 的 `robot_description` 改由 `demo.launch` 原生加载
   （canonical xacro 已含 tcp_link，overlay 原文档写明"一旦并入即退役"）。
2. `setup_arm.launch` 给 lift_joint_bridge 显式传 `joint_name`（自文档化）。

### 四、验证（offboard，2026-10-13 Docker Noetic 实测）

| 项 | 结果 |
|---|---|
| `dh_gripper_driver`（含 temp 修复）单独编译 | ✅ 通过（1 个编码类警告，vendor 源码常态） |
| `xarm_controller`（含 temp xarm_hw）单独编译 | ✅ 通过（Docker 内 SDK 为树内 vendor 包，可编；真机 `xarm_api` 仍只能在真机验证） |
| simulation profile 全量构建（`scripts/build_ros1_workspaces.py --profile simulation`） | ✅ 25+3 包全部成功 |
| canonical xacro XML 解析 + tcp_link/tcp_joint/lifting_joint 上限断言 | ✅ tcp_link 0.170m；lifting_joint 0..0.7 保留（Isaac 合同） |
| 离线测试 `src/tasks/tests/run_all.sh` | ✅ exit=0，全部通过（实测 119 断言）；`run_all.sh` 同时支持真机布局与 Docker 仓库布局回退 |
| 假臂 headless 冒烟 | ✅ 见下第六节 |

### 六、假臂 headless 冒烟实测（fake_plane_target，无 RViz/jsp GUI）

| 检查 | 结果 |
|---|---|
| 节点集 | ✅ 仅任务/可视化/demo 链；无任何 xarm/lift/相机/底盘驱动节点 |
| `base_link→tcp_link` TF | ✅ 可解析（tcp_link 并入 canonical 模型后全链通） |
| 干跑 `go` | ✅ 回「已触发规划（干跑模式）」，fake 控制器流静止（1→1） |
| 执行 `go` | ✅ 运动完成 ~2.1 s，`tcp_link` 回读 (0.335, 0, −0.108) in link_base，**距命令点 0.1 mm、工具轴 0.1° 偏差** |
| 安全区约束 | ✅ 每次运动都打出生效盒 x[0.106,0.565] y[±0.25] z[0.028,1.298]（来自 /safety_plane/box） |
| `home` | ✅ 回 initial，运动带约束完成 |
| `grasp`（无夹爪驱动） | ✅ 受理并播 `busy\|…`，随后 `fail\|/gripper/ctrl 没有订阅者：夹爪驱动没起…`；slash 干跑下未见任何闭合下发 |
| busy 守卫 | ✅ home 动作未完成时调 grasp 被拒（「运动/夹取中，已忽略本次夹取」）——控制器独占语义正确 |

> 复测注意：`move_group/fake_controller_joint_states` 的流**要在触发前挂 `rostopic echo`**，
> 触发后才挂只能数到静止基线（本轮冒烟脚本第一版就犯了这个时序错——按服务 success 判断会
> 误读成"没动"，以 `motion done` 日志 + TF 回读为准）。

> 踩坑记录：改 `lift_joint_bridge.py` 时一度把话题名多打成 `LiftingMotorStatePub`（正确为
> `LiftMotorStatePub`）。离线测试对该订阅是打桩的所以抓不到—— review 时对照 AGENTS 坑位才
> 发现。教训：**改共享常量要跟小节文档对照，不能只看局部 diff**。

### 五、待办 / 下一步

1. ✅ Docker 内全量 `catkin build`（simulation profile）+ `bash src/tasks/tests/run_all.sh`
   （✅ exit=0，119 断言全过）。
2. ✅ 假臂 headless 验收（见第六节：干跑/执行/home/安全区约束/busy 守卫/grasp 边界全过）。
3. 夹爪假驱动 shim（订阅 `/gripper/ctrl`，回报 `/gripper/states`），让 `plane_target/grasp`
   端到端跑通 ——双侧共用同一份 shim（sim/ros1_ws 内），见该文档 Follow-on boundaries。
4. 真机部署顺序：`scripts/package_agilex_workspace.py` → U 盘 → 真机
   `catkin build` → `roslaunch agx_bringup hardware.launch` → 先干跑
   `plane_target_execute:=false` → 升降 ≥0.2 m → 实动（真机 xarm_api 编译只有真机能证明）。

---

## 2026-09-30 —— 夹取：控制台加「夹取」按钮（移到红点 + 闭合），闭合位置可调

### 一、目标

"夹爪移动到红球处后夹取，以后会实现自动化流程，目前需要在 UI 控制台中加入夹取按钮，
并且夹爪闭合参数可输入调节以实现夹取不同大小物品。"

三个明确要求 + 一个隐含要求：
1. 控制台加「夹取」按钮
2. 夹爪闭合参数可输入调节，适配不同大小物品
3. **以后做自动化流程** → 这决定了架构（见第三节）
4. 隐含：不能破坏现有的安全约束（黄色安全区、干跑开关、急停）

### 二、完成内容

**新增 `plane_target/grasp` 服务（`agx_plane_target`，任务层）**：一次完成
「(可选)张开 → MoveIt 移到红点 → 闭合到目标值 → 回报实际停住位置」。
进度走 `plane_target/grasp_status`（latched `"<state>|<text>"`）。

**控制台新增「夹取」行**（`joint_slider_panel.cpp`）：`闭合位置` 输入框（0~1000，默认 0）
+ 「夹取（移到红点后闭合）」按钮 + 状态标签。执行期间按钮自动禁用，结束后自动重新同步夹爪滑条。

**参数每次夹取重读**（`~grasp_position` 等）：换物品只改数字，不重启任何节点。
控制台点按钮前 `setParam` 再调服务——和已有的 `~rpy_deg` 同一套路。

### 三、关键设计取舍（详见 `decisions.md` D21）

| 问题 | 选择 | 理由 |
|---|---|---|
| 一次点击 vs 点两次 | **一次点击** | 用户要在此基础上做自动化，**自动化的原子单位必须是服务**；且 `busy` 全程持有，防止 GO 插进"移动完还没闭合"的窗口把半闭合的夹爪抬走 |
| 闭合参数暴露什么 | **只暴露位置**（用户指定） | 力度固定 50（与夹爪滑条一致），留 `~grasp_force` 参数可 rosparam 调 |
| 到位怎么判 | **位置读数连续 N 次不变** | **不读 `grip_state`**——M4 至今没实测，猜厂商枚举等于把猜测写进安全判断。"停住 + 停在哪"是可观测事实，三种情况都能如实描述 |
| 参数走 srv 还是 param | **param** | 分层规则禁止 `start` 依赖 `tasks`；自定义消息要么违规要么再加中立 msg 包，此时不值得。正式契约留给 `agx_task_msgs` |
| 进度怎么回传 | **`std_msgs/String`** | 同上。自定义消息对三态进度是重依赖；`Trigger.message` 本来就是这个用法 |

**为什么"到位"要有 0.25s 起步窗口**：指令刚发出时手指还没动，那段"静止"是假的。
不加窗口会立刻误判到位——测试里专门用"延迟 0.3s 才动"的假设备守着这条（`test_grasp.py` 第 3 组）。

**AG95 没有 cancel 输入**，所以"急停中断闭合"= **重发它已到达的位置**（保持）。

### 四、验证

| 层次 | 手段 | 结果 |
|---|---|---|
| 离线逻辑 | `src/tasks/tests/test_grasp.py`（11 组 / 60 断言） | ✅ 全过。含到位判定、起步窗口、急停中断、超时、状态流中断、干跑不闭合、度弧度回归 |
| 全套离线 | `bash src/tasks/tests/run_all.sh` | ✅ 115 断言全过（新增后） |
| 跨进程契约 | 隔离 roscore(11399) + fake AG95 | ✅ 服务名/参数/状态格式都对；**干跑下 fake 收到 0 条 `/gripper/ctrl`** |
| 真实路径夹爪阶段 | 同上，`enable_arm=true` | ✅ 张开确实下发 1000 并等到位；MoveIt 缺失时**如实报 fail 且不下发闭合** |
| 编译 | `catkin_make --pkg agx_xarm_bringup` | ✅ 通过 |
| 控制台离屏启动 | `QT_QPA_PLATFORM=offscreen` | ✅ 正常启动无崩溃；新行字符串已在二进制中 |
| **真机端到端** | — | ⏸ **未做**（需人工清场 + 物理急停在手边） |

验证过程中测试**发现并修掉两个真问题**（不是测试写错）：
1. 干跑路径**不播 `busy`**，控制台按钮会一直亮到流程结束
2. MoveIt 异常只丢英文 `Unable to construct robot model`，控制台一行放不下也没 actionable 提示

### 五、踩坑记录（避免重犯）

- **`pkill -f plane_target_marker` 误杀了用户正在跑的节点**。`rosrun` 展开后是
  `devel/lib/agx_plane_target/plane_target_marker.py`，宽模式必中。已用
  `pkill -f "plane_target_marker.py _enable_arm"` 限定，并已恢复节点。已写进 `AGENTS.md` 坑位
- **别把"服务立刻返回"当成"动作已完成"**。`plane_target/*` 全部立即返回、动作在节点线程里跑。
  异步进度只能靠话题回传
- **我自己的 3 条测试断言写错了**（以为 `~rpy_deg` 改 90 度会让工具轴歪 90 度——实际 yaw 是绕
  工具轴旋转，倾角**就该**保持 0°，变的是"拧向"）。修正方式是**先算一遍数学再写断言**，
  并加一条"对照组"断言证明这条断言不是恒真的（把 180 当弧度会歪 53.2°）

### 六、改动文件清单

| 文件 | 改动 |
|---|---|
| `src/tasks/agx_plane_target/scripts/plane_target_marker.py` | grasp 服务 + 夹爪读写 + `_motion` 拆成 `_execute_motion`(不管 busy)/`_motion`(管 busy) |
| `src/tasks/agx_plane_target/package.xml` | 补 `std_msgs` / `dh_gripper_msgs` |
| `src/start/agx_xarm_bringup/src/joint_slider_panel.{h,cpp}` | 「夹取」行（输入框/按钮/状态标签）+ 按钮禁用与滑条重同步 |
| `src/start/agx_xarm_bringup/package.xml` | 补 `std_msgs` |
| `src/start/agx_xarm_bringup/launch/setup_arm.launch` | 注释补 grasp 参数说明 |
| `src/tasks/tests/test_grasp.py` | **新增**，11 组 60 断言 |
| `src/tasks/tests/run_all.sh` | 加入 `test_grasp.py` |
| `src/tasks/doc/decisions.md` | **D21** |
| `src/tasks/README.md`、`AGENTS.md` | 接口表 / 状态表 / 坑位 |

### 七、待办 / 下一步

1. **重启 `roslaunch agx_xarm_bringup setup_arm.launch`**（Python 免编译，但控制台 Qt 二进制要重启才生效）
2. **重启后自查**：`rosnode info /plane_target_marker` 的 Services 里应有 `/plane_target/grasp`；
   控制台「夹取」行应从灰变亮（不亮 = 节点没重启到新版）
3. **真机端到端**（先干跑再上真机）：
   - `plane_target_execute:=false` 干跑一次，确认**只规划、夹爪不动**
   - **抓取前把升降柱升到 ≥0.2 m**（全降位时红点在臂底下方 108 mm，规划会超时）
   - 真机点「夹取」，确认状态依次走过 准备 → 张开 → 移动 → 闭合，最后报「停在 XXX」
4. **补 M4（`grip_state` 语义实测）**：这是实现"夹住 vs 夹空"判断的前置条件。
   当前只报实际停住位置，操作者自己判断
5. 自动化流程（本需求的后续）：`rosservice call /plane_target/grasp` 已是原子单位，
   状态机可直接复用；届时再把接口正式化到 `agx_task_msgs`（见 `interface.md`）

---

## 2026-09-29 当日汇总（8 项，详细记录见下方各小节，最新在上）

**一句话**：把"平面红点到位"从"能用"推到"可验证、可复现、受约束"——修了两个**静默算错**的
bug（度/弧度、RViz 工具位），加了轨迹安全区约束与姿态自检，标定了 TCP 与安全区高度，
并消掉了两处硬编码重复。

| # | 起因（用户看到的现象） | 根因 | 改动 | 状态 |
|---|---|---|---|---|
| (一) | 升降柱断电重启后高度归零 | 位置基准靠"下降压限位+清零"建立，断电丢失 | 加坐标系列定门禁 + 修限位保护 | ✅ 已解决，真机回归通过 |
| (二) | 夹爪不竖直向下（歪 53°） | **`~rpy_deg` 把度传给了吃弧度的 `quaternion_from_euler`**，不报错 | `math.radians()` + 启动/执行后自检日志 | ✅ 代码已验证（0.0°）；⏸ 实物待水平尺复核 |
| (三) | 轨迹/复位时机械臂"危险举动" | GO 只给终点位姿；「回到初始位置」更是裸关节轨迹直发控制器 | MoveIt path constraint 锁末端全程在黄盒内 + `plane_target/home` 改走 MoveIt | ✅ 已验证约束确实绑定；⏸ 待加载 |
| (四) | 红点难抓 | 45mm 球在正常视距只有几像素 | 加 100mm 半透明抓取手柄 + 提示文字 | ✅ 已实现；⏸ 待加载 |
| (五) | **鼠标完全选不中/拖不动红点** | **RViz 激活工具是「移动相机」**（`Tools:` 首位在它），marker 不响应 | `arm_display.rviz` 改回 `rviz/Interact` 首位 | ✅ 配置已修；⏸ 待加载或点 Interact 按钮 |
| (六) | 黄区底面偏高，夹爪够不着甲板 | 盒底 98mm 偏高 | 底面降 70mm（0.098→0.028，顶面不变）+ **盒子数字单一真源**（`/safety_plane/box`） | ✅ 已验证两处一致；⏸ 待加载 |
| (七) | `tcp_link` 离指尖远 | 旧实测值 0.230 m **探过头 60mm** | `tcp_joint` 改 **0.170 m**，M2 同步结案 | ✅ FK 验证只沿工具轴挪 60mm；⏸ 待加载 |

### 合并待办（去重后 3 件，按顺序做）

1. **重启一次 `roslaunch agx_xarm_bringup setup_arm.launch`**，今天 7 项改动一次全部生效
   （Python 免编译；**唯一需要编译的是控制台 `joint_slider_gui`**，已在 (三) 编译过）。
   暂时不想重启 RViz 的话，拖不动红点时点工具栏最左边的 **Interact（手掌）** 按钮即可
2. **重启后按这几行自查**（任一行不符 = 没加载成功）：
   - `[plane_target] goal orientation rpy=(180.0, 0.0, 0.0) deg ... 0.0 deg off vertical-down`　← (二)
   - `[plane_target] workspace keep-in: ON (tcp_link), box from /safety_plane/box, bottom z=0.028 top z=1.298`　← (三)(六)
   - `safety box ... z[0.028,1.298] ... published to /safety_plane/box`　← (六)
   - `rosrun tf tf_echo base_link tcp_link` 数值应比改前**小 60 mm**　← (七)
3. **真机端到端验证**（先干跑再上真机）：
   - `plane_target_execute:=false` 干跑一次，确认降底面 + 缩 TCP 后规划仍能过
   - **抓取前把升降柱升到 ≥0.2 m**（全降位时红点只在臂底下方 108 mm，规划会超时够不着）
   - 夹爪朝下停到红点后，用尺子量**真实指尖离甲板面**的高度，应与红点高度（甲板面上方 100 mm）
     基本一致——这是 M2 一直挂着没做的端到端验证，做完 TCP 标定链就完整
   - 夹爪若仍不竖直，才轮到"姿态相关的关节零位偏差"（TF 照不出来，需水平尺现场复核）

### 今天最值得记的一条教训

**"能跑通"不等于"算对了"。** (二) 和 (五) 都是**不报错的静默错误**：一个是度当弧度喂给欧拉角
函数（算出合法但错 53° 的姿态），一个是 RViz 工具位不对（marker 发布一切正常就是不响应）。
两者都只能靠**主动加观测点**才暴露——前者加了姿态自检日志，后者靠"横向对照 40+ 个 .rviz 配置"。
反过来 (三)(四)(六) 三个功能型改动都不如这两个 bug 关键：
**先让系统"可被证伪"，再加功能。**

---

## 2026-09-29(二) —— 平面红点：夹爪朝向排查（目标姿态本已"工具轴竖直向下"）+ 加姿态自检日志

### 一、目标

用户反馈：「能到红点位置，但夹爪位姿不对，希望夹爪 `tcp_link` 的蓝轴（Z）竖直向下」。

### 二、根因：度/弧度没转换（`quaternion_from_euler()` 吃弧度）

**`~rpy_deg` 参数名和文档都写的是"度"，但代码直接把它当弧度传给了
`quaternion_from_euler()`**（`tf.transformations` 的约定是弧度，没有报错、也不报错，
只是安静地算出另一个合法姿态）：

- `180` 被当成 180 **弧度** = 180 − 28×2π = **233.3°**
- 于是 `tcp_link` +Z 不是 `(0,0,−1)`，而是 `(0, +0.80, −0.60)`，
  **离竖直向下 53.2°** —— 正好是用户看到的"夹爪是斜的"
- 证据（GO 日志，`goal rpy=(180.0, 0.0, 0.0) deg -> tcp +Z = (-0.00, +0.80, -0.60) ... 53.2 deg off`）

**这个 bug 从功能上线就一直在**（原代码 `quaternion_from_euler(*rpy_deg)`），
之前没暴露是因为"位置对、姿态斜"很容易被当成 URDF/实物零位偏差，
而 TF 由 `joint_states` 推出、天然与命令一致，**照不出命令本身的错**。

排除掉的其它可能（都验证过）：

| 怀疑 | 结论 |
|---|---|
| 工具轴不是 Z | 不是。`xarm_with_ag95` 与 `tcp_joint` 的 rpy 都是 `0 0 0`，AG95 手指关节在法兰上方 0.106 m 沿 +Z 展开（`dh_robotics_ag95_model_macro.xacro`），`tcp_link` +Z = 指尖方向 |
| 规划不出来 / 运动没收敛 | 不是。探针脚本实测 `link_base (0.235, 0.080, -0.408)` + 姿态约束 **0.1 s 规划成功**，`execute()` 返回 True |
| URDF 与实物安装/零位偏差 | 不是。用户现场确认空闲位姿实物夹爪"水平指向车头"，与 TF 一致 |
| 提前停止 / 被急停 | 只发生过 1 次 `execution canceled/stopped`，不是主因 |

### 三、改动（`agx_plane_target/scripts/plane_target_marker.py`）

| 改动 | 说明 |
|---|---|
| **修 bug**：`~rpy_deg` 先 `math.radians()` 再进 `quaternion_from_euler()` | 修完 `180,0,0` → `tcp +Z = (0,0,−1)`，离竖直向下 **0.0°**（走真实 `_goal_quat()` 逐组验证过 9 组输入） |
| **启动自检**：启动就打一行命令姿态 | 倾角 > 5° 时追加 ` <== NOT pointing down, check ~rpy_deg`，**不用点一次 GO 就能发现配错** |
| `~rpy_deg` 改成**每次 GO 实时读取** | 以前只在启动读一次；现在 `rosparam set /plane_target_marker/rpy_deg "180,0,90"` 后直接点 GO 即生效，不用重启（绕自身轴拧向靠 yaw，已验证 `180,0,±90` 不影响 Z 朝下） |
| GO 时打印命令姿态 | `goal rpy=(...) -> tcp +Z = (...) in link_base, X deg off vertical-down` |
| 执行完**回读 TF**（新增 `_lookup_tcp()`） | `tcp_link now (...), X mm from the commanded point | X=(...) Y=(...) Z=(...) | tool axis X deg off vertical-down` |
| 新增纯函数 `_axes_of()` / `_tilt_deg()` | 由四元数算出三根轴在父系里的指向，以及该轴离"竖直向下"的夹角 |

`_lookup_tcp()` 要兼容两种返回形状：Noetic 的 tf1 shim 有时给
`((x,y,z),(qx,qy,qz,qw))`、有时给**扁平 7 元组**（第一版按前者写，实测报
`'float' object is not subscriptable`）。

### 四、待办

- ⏸ **重启 launch**（Python 免编译，但节点进程要重启才加载新代码；roslaunch 不会自动拉起被杀掉的节点），
  确认启动日志那行显示 `180.0, 0.0, 0.0 deg -> ... 0.0 deg off vertical-down`，
  再点一次 GO，回读那行也应显示 `0.0 deg`
- ⏸ 若修完实物仍不竖直，才轮到"姿态相关的关节零位偏差"（TF 照不出来，需水平尺现场复核）

---

## 2026-09-29(三) —— 轨迹安全区约束：末端全程锁在黄色盒子里 + 「回到初始位置」改走 MoveIt

### 一、目标

1. 抓取时夹爪仍不竖直向下（上一轮修了度/弧度 bug，但**用户测的是旧节点**，见下）
2. 「路径规划或者复位时有时候机械臂会做出危险举动」——**不希望运动轨迹超出黄色安全区域**

### 二、问题 1：用户测的是修复前的节点

节点进程 `13:29:01` 启动，脚本修复写入时间 `13:34:02`——**修复根本没被加载**。
日志里 `goal rpy=(180.0,0.0,0.0) deg -> 53.2 deg off` 和 `'float' object is not
subscriptable` 两行都是修复前的输出格式，可据此判断节点版本。教训见「三之 1」。

### 三、问题 2：安全区约束（本次主体）

#### 1. 之前为什么没有约束

- `plane_target` 的 GO 只给了终点位姿，MoveIt **怎么走都行**（关节空间插值 + 避障自己选）
- 控制台「回到初始位置」更是**一条裸关节轨迹直发控制器**
  （`joint_slider_panel.cpp: sendTarget()` 直接 `command_pub_.publish(traj)`，
  单点轨迹 + 按最大关节差算时长），**没有规划、没有碰撞检查、没有工作空间限制**——
  这就是"复位时危险举动"的来源

#### 2. 实现：MoveIt path constraint（`keep-in box`）

`moveit_msgs/PositionConstraint` 的 `constraint_region` 里放一个 `BOX` primitive，
含义是"这个点必须待在这个盒子里"。MoveIt 在**每个采样状态**上重新检查路径约束，
所以约束的是**整条扫掠轨迹**，不只是终点。

盒子 = 黄色暂定安全区（`deck_top` 系，静止不随升降），数字与 `safety_plane_viz.py`
的 `PLANE` 完全一致：`x[0.106,0.565] y[±0.250] z[0.098,1.298]`
（中心 `(0.3355,0,0.698)`，尺寸 `0.459×0.50×1.200`）。

| 参数 | 默认 | 说明 |
|---|---|---|
| `~keep_in_zone` | `true` | 关掉就退回无约束（会 `logwarn` 明确告知） |
| `~zone_margin` | `0.0` | 六面内缩量。**注意红点 z=0.100 只比盒底 0.098 高 2 mm**，内缩 >2mm 会让红点不可达 |
| `~zone_links` | `tcp_link` | 可加 `link6`/`link5` 收紧扫掠范围（实测这三个 link 当前都在盒内，加进去也规划得通） |
| `~home_named_state` | `initial` | `plane_target/home` 用的 SRDF 命名位姿 |

**只约束末端、不约束整条臂**：臂基座 `link_base` 在 x=0，而盒子从 x=0.106 起，
整条臂在物理上就装不进盒子。

**起点在区外的兜底**：MoveIt 路径约束对**起点也生效**，所以手臂停在区外时
所有规划都会莫名超时。`_zone_bounds()` 检测到被约束的 link 在区外时，
把盒子**最小化扩一个刚好包住它**，并 `logwarn` 说明"约束已临时扩大"——
手收回黄区内就自动恢复严格约束。

#### 3. 「回到初始位置」改走 MoveIt

新增服务 `plane_target/home`（`std_srvs/Trigger`）：用 `set_named_target("initial")`
**带安全区约束**规划并执行。SRDF 的 `initial` = `(0,0,0,0,-1.5708,0)`，
与面板里 `kInitialArm` 硬编码的值完全一致，不用维护两份。
面板（`joint_slider_panel.cpp: goToInitialPose()`）改为**优先调这个服务**；
服务不可用时才退回原来的直发方式，并在状态栏明写「未经规划，路径不受黄色安全区约束」。

#### 4. 验证（全部 plan-only / 干跑，未驱动机械臂）

| 测试 | 结果 |
|---|---|
| 目标 x=0.00（**黄区外**，柱正上方）+ 约束 | **超时失败**（12s） |
| 同一目标 **无**约束 | **0.7s 成功** ← 证明约束真的绑得住，不是摆设 |
| 目标 x=0.30（区内）+ 约束 | 0.2s 成功 |
| 命名位 `initial` + 约束 | 0.1s 成功（当前关节与 `initial` 完全一致，故只 2 个轨迹点） |
| 红点 (0.30, 0.00) + 约束，节点干跑路径 | plan OK，25 waypoints |
| 起点区外自动扩框（把 `link_base` 塞进约束） | bounds 自动从 `x[0.106,…]` 扩到 `x[0.000,…]` ✔ |

### 四、当前实机状态（顺手查到的）

- 机械臂**正停在 `initial` 位姿**（6 关节与 SRDF `initial` 逐个相同，最大差 0.0000 rad）
  —— 说明用户确实按过「回到初始位置」，也就是撞上了旧的直发路径
- **`lift_joint = 0.0`，升降柱在全降位**。此时红点只在臂底下方 **0.108 m**，
  规划到部分红点会 15s 超时（够不着/折叠不开）。要顺畅抓取建议升到 **≥0.2~0.3 m**
  （0.3 m 时红点在臂底下方 0.408 m，历史记录里 7 次成功都是那个高度）

### 五、待办

- ⏸ **必须重启 launch** 才能同时拿到：新 Python（度/弧度修复 + 安全区约束 + `home` 服务）
  和**新编译的控制台**（`catkin_make --pkg agx_xarm_bringup` 已完成，但二进制要重启进程）
- ⏸ 重启后确认启动日志两行：`goal orientation ... 0.0 deg off vertical-down`、
  `workspace keep-in: ON (tcp_link), exactly the yellow box`
- ⏸ 抓取前先把升降柱升到 ≥0.2 m
- ⏸ 若仍嫌轨迹太野，把 `~zone_links` 改成 `tcp_link,link6,link5`（更严）
- ⏸ 关节滑条**手动拖动无法加这种约束**（那是直接下发的），只能靠速度滑条限制快慢

---

## 2026-09-29(七) —— TCP 标定修正：`tcp_joint` 0.230 → 0.170 m（M2 复测落地）

### 一、需求

「tcp_link 现在离机械臂指尖有点远，我希望调近 60mm」。确认方向 = **旧的 tcp_link 探过头 60mm**
（真实指尖在它上方 60mm 处），故 `0.230 → 0.170`，并同步更新 M2 记录。

### 二、改动

| 文件 | 改动 |
|---|---|
| `ranger_xarm6_lifting_2431.xacro:75` | `tcp_joint` origin `xyz` `0.0 0.0 0.230` → **`0.0 0.0 0.170`**；上方注释重写（实测来源、复测结论、历史教训、"别再据模型 FK 改这里"） |
| `measurements.md` M2 | 表格改为实测 **0.170**（2026-09-29 确认）/ 张开未复测；"模型短 83mm"标为**推断作废**；新增"仍未逐项确认"小节（参照面未留证、张开值、端到端） |
| `decisions.md` D12 / D16 | D12 结论与表格更新为 0.170；D16 标题改为「✅ 推断作废」，证据改为 0.170 vs 0.147 |
| `interface.md` | TCP 偏移 0.230 → 0.170，存疑警告改为已作废 |
| `kinematics_ik_plan.md` | 运动链表第 10 段 `0 0 0.230` → `0.170`；风险表第 1 条标为已解决 |
| `tasks/README.md` | 改动追踪表 4 处同步 |
| `AGENTS.md` 坑 16 | `tcp_link` 说明 0.230m → 0.170m |

**连带结论（重要）**：AG95 网格 FK 最低点 0.147 m vs 实测 0.170 m，**只差 23 mm**，所以 2026-09-28
存疑的那条"模型比实物短 0.083 m"是**错的**——错的是**实测值本身**（怀疑初测量的参照面是
法兰背面/安装面而非 URDF `link6` 原点平面，差 60 mm 量级）。残余 23 mm 方向是**网格略短**
（碰撞体偏保守），不是量级问题。

### 三、验证

1. **xacro 展开通过**（真编译期校验）：`xacro ...2431.xacro` → 1050 行 URDF，
   `tcp_joint` 为 `xyz="0.0 0.0 0.170"`
2. **新旧 URDF 逐行 diff：只有 1 行**（`tcp_joint` 的 origin），其余完全一致
3. **FK 验算**（`urdf_parser_py`，喂真实 `/joint_states`）：
   ```
   link6                       |Δ|=  0.0 mm   不变
   dh_robotics_ag95_base_link  |Δ|=  0.0 mm   不变
   tcp_link                    |Δ|= 60.0 mm   方向 (-0.66,+0.34,+0.67) = 沿工具轴退回法兰
   ```
   即"只沿工具轴挪 60mm，其它一律不动"
4. `py_compile` 通过；全工作区已无过期的 0.230 / 0.220 TCP 引用

### 四、影响与待办

**影响**：末端参考点变了 → MoveIt 规划目标、TF、`plane_target` 落点全部跟着变。平红点命令的是
"指尖中点"，所以**夹爪会停得比之前高 60 mm**（之前指尖在红点上方 60mm 处悬空）。

- ⏸ **重启 launch 即可**（xacro 在启动时展开，**不需要 catkin_make**）
- ⏸ 重启后验证：`rosrun tf tf_echo base_link tcp_link` 数值应比改前小 60 mm；
  RViz 里 `tcp_link` 与夹爪网格底部距离应 ≈ **23 mm**
- ⏸ **端到端**（最值得做，M2 里也标着没做）：夹爪朝下停到平红点，用尺子量真实指尖离甲板面
  的高度，应与红点高度（甲板面上方 100 mm）基本一致
- ⏸ 仍挂着的 M2 遗留：参照面未拍照留证、**张开状态值未复测**（旧的 0.220 是与作废的 0.230
  配对的，不可沿用）、AG95 网格未修正（短 23 mm，方向保守）

---

## 2026-09-29(六) —— 黄色安全区底面下降 70mm + 消除盒子数字的重复定义

### 一、需求

1. 「黄色安全区底部高度有点低……现在最低位置有点高，下降 70mm」——把盒底从
   `deck_top` +0.098 降到 +0.028，让夹爪能更靠近甲板作业
2. 消除重复定义：盒子数字原来在**两个文件里各写一份**

### 二、改动

#### 1. 降底面（`safety_plane_viz.py:39`）

盒子规律：`底面 = pos.z − size.z/2`。降底面 d 而**顶面不动**（盒子变高而非平移）：

| | 旧 | 新 |
|---|---|---|
| `size.z` | 1.200 | **1.270** |
| `pos.z` | 0.698 | **0.663** |
| 底面 | 0.098 | **0.028**（−70mm） |
| 顶面 | 1.298 | 1.298（不变） |

红点高度 `PLANE_Z = 0.100` 不动，从此比盒底高 **72mm**（原来只高 2mm）。
副作用（好的方向）：`~zone_margin` 的可用范围从 2mm 放宽到 72mm，
不再一不小心内缩就让红点不可达。

#### 2. 单一真源（消除重复定义）

```
safety_plane_viz.py 的 PLANE ──启动时 set_param──>  /safety_plane/box
                                                        |
                                          每次运动重读 ↓
                                          plane_target_marker.py 的
                                          MoveIt keep-in 约束
```

- `safety_plane_viz.py` 新增 `PLANE_PARAM = "/safety_plane/box"` 与
  `publish_box_param()`，写入 `{center, size, frame_id, ns}` 并打一行日志
- `plane_target_marker.py` 删掉 `ZONE_SIZE` / `ZONE_CENTER` 硬编码，新增 `_zone_box()`
  读该参数；读不到才退回 `FALLBACK_ZONE_*` 并 `logwarn_throttle`（两个节点并行启动，
  谁先谁后都无所谓，因为每次运动都重读）
- **方向符合分层硬规则**：`tasks → start` 允许、反向禁止；param server 正是 AGENTS
  认可的松耦合字符串接口。`_zone_bounds()` / `_zone_constraints()` 改为显式接收
  `size`/`center`/`bounds`，不再引用模块级常量

### 三、验证

1. 两个文件独立算出同一个盒子：`z[0.028,1.298]`，底面/顶面一致；相对旧值 −70mm / ±0mm
2. 起真 `safety_plane_viz` → `rosparam get /safety_plane/box` =
   `{'center': [0.3355, 0.0, 0.663], 'size': [0.459, 0.5, 1.27], ...}`
3. 真 `_zone_box()` 读到该参数（`source=/safety_plane/box`），`bounds` 推出
   `x[0.106,0.565] y[±0.250] z[0.028,1.298]`，生成的 `PositionConstraint`
   尺寸 `(0.459,0.500,1.270)` 中心 z `0.6630`
4. 顺手验证兜底分支：把 param 读成 `None` 时回退内置常量并 `logwarn`，结果一致
5. 两个文件 `py_compile` 通过

### 四、待办

- ⏸ **重启 launch**：两个节点都是 Python（免编译），但要重启进程才加载新数字
- ⏸ 重启后看两行确认：
  - `safety box x[...] y[...] z[0.028,1.298] (frame deck_top) published to /safety_plane/box`
  - `[plane_target] workspace keep-in: ON (tcp_link), box from /safety_plane/box, bottom z=0.028 top z=1.298`
  出现 `built-in fallback` 说明 `safety_plane_viz` 没起或参数没写上
- ⏸ 建议先 `plane_target_execute:=false` 干跑，确认降底面后规划仍能过

---

## 2026-09-29(五) —— 根因：RViz 激活的工具是「移动相机」，不是「交互」

### 一、现象

用户两次反馈「红点无法用鼠标选中或拖动」。marker 侧一切正常：

```
/plane_target/update_full:  drag_plane(MOVE_PLANE, always_visible, 45mm)
                            grab_handle(MOVE_PLANE, always_visible, 100mm)
                            hint(NONE, 文字)
/plane_target/feedback:  发布者 /arm_rviz  ->  订阅者 /plane_target_marker   双向通
Plane Target 显示项: Enabled=true, Update Topic=/plane_target/update
节点无重名、无残留 RViz 进程、节点日志无报错
```

### 二、根因

`src/start/agx_xarm_bringup/rviz/arm_display.rviz` 里

```yaml
Tools:
  - Class: rviz/MoveCamera      # 首位 = 启动时激活的工具
  - Class: rviz/Interact
```

**RViz 加载配置时激活 `Tools:` 列表的首位**。用「移动相机」时，左键拖动 = 转视角，
interactive marker 的拾取/拖动**根本不会触发**——所以既选不中也拖不动。
而 RViz **存配置时会把当前激活的工具写到首位**，等于一次误存就把坑固化了。

**对照证据**：全工作区 40+ 个 `.rviz` 配置**全部**是 `rviz/Interact` 在首位，
只有 `arm_display.rviz` 例外（说明是某次用 MoveCamera 时误存盘造成的）。

### 三、修复

`arm_display.rviz` 的 `Tools:` 改为 `rviz/Interact` 首位（保留 `Hide Inactive Objects: true`），
并就地加注释说明"首位即激活项"以防回归。YAML 已用 `yaml.safe_load` 复核：

```
Tools 顺序: ['rviz/Interact', 'rviz/MoveCamera', 'rviz/Select', 'rviz/FocusCamera', 'rviz/Measure']
InteractiveMarkers: Plane Target Enabled=True Topic=/plane_target/update
```

### 四、教训（已写进 AGENTS 坑 14）

排查"RViz 里 marker 看得到但点不动"**先查 `Tools:` 首位**，而不是去怀疑 marker 代码。
本次因为合成反馈被 client_id 保护丢弃（坑 19），先后怀疑过"反馈链路断了""抓取区太小"，
都走了弯路——最后是"40+ 个 .rviz 只有它不一样"这个横向对照直接定位。
临时补救不用重启：点工具栏最左边的 Interact（手掌图标）。

### 五、待办

- ⏸ **重启 RViz**（或整个 launch）让新配置生效；不想重启就点工具栏的 Interact 按钮
- ⏸ 重启后立刻试：拖动红球应只移动红点，原地短按才执行

---

## 2026-09-29(四) —— 红点鼠标拖动手感：加大透明抓取手柄 + 操作提示文字

### 一、需求

「把红点发布为可以用鼠标直接拖动，同时也要保留控制台的方向控制」。

### 二、核查结论：机制本来就在，问题是不好抓

`/plane_target/update_full` 实测（用户已重启到新版本）：

```
name: "target"  frame: deck_top
control drag_plane  interaction_mode: 4(MOVE_PLANE)  always_visible: True
                    SPHERE scale=0.045
/plane_target/feedback:  发布者 /arm_rviz  ->  订阅者 /plane_target_marker  （双向通）
/plane_target/nudge:     控制台方向键 -> 节点（保留，未改动）
```

也就是说**鼠标拖动与控制台方向键两条路都是通的**。真实短板是：
**45mm 的红球在正常视距下只有几个像素，鼠标很难点中**——所以"感觉拖不动"。

### 三、改动（`agx_plane_target/scripts/plane_target_marker.py`）

一个 marker 挂三个 control：

| control | 模式 | 内容 | 作用 |
|---|---|---|---|
| `drag_plane` | `MOVE_PLANE` | 45mm 不透明红球（原有） | 拖动 + 短按执行 |
| `grab_handle` | `MOVE_PLANE` | **100mm 半透明球（alpha 0.22）** | **加大抓取目标**，专供鼠标抓 |
| `hint` | `NONE` | `TEXT_VIEW_FACING` 文字 | 提示「拖动移动 · 短按执行 · 控制台方向键也行」，`NONE` 模式**不可拾取**，不干扰拖动 |

**为什么两个 control 都是 `MOVE_PLANE`**：同一种交互模式抓中哪个都一样，结果无差别；
**绝不能**再叠一个 `BUTTON`（RViz 拾取顺序不确定，拖动和点击会互相抢，见坑）。

颜色反馈（待命红 / 运动中黄 / 成功绿 / 失败橙）对两个球同时生效，
`_set_dot_color()` 重建整个 marker，所以提示文字和手柄在颜色变化时都保留（已单测四种状态）。

### 四、坑：`InteractiveMarkerServer` 会拒绝"外来"反馈

想用合成反馈做自动化测试时踩到：`processFeedback()` 里有

```python
if marker_context.last_client_id != feedback.client_id and (now - last_feedback) < 1.0:
    return   # 静默丢弃
```

只要 **1 秒内有另一个 client（这里是 RViz）** 发过反馈，外部构造的
`InteractiveMarkerFeedback` 会被**静默丢掉**。当时正好是用户在 RViz 里拖红点，
测出来的"滞后一拍"其实是**用户自己的操作**，差点误判成代码 bug。
→ 想离线测拖动链路，必须先确认没有别的 client 在动这个 marker（停 RViz 或换一个时间段）。

### 五、待办

- ⏸ **重启 launch** 生效（Python 免编译，节点进程要重启）
- ⏸ 确认：红球变大发亮、周围有一团半透明红晕、上方有提示文字；拖动仍然只在黄区内平移
- ⏸ 若嫌 100mm 手柄太大/太显眼，调 `GRAB_SIZE`（默认 0.100）

---

## 2026-09-29(一) —— 升降柱断电后高度归零：定位根因 + 加坐标系列定门禁 + 修限位保护失效

> **状态：已解决并通过真机全链路回归**（见「六之三」）。决策依据见 `src/tasks/doc/decisions.md` D20，
> 实测数据见 `src/tasks/doc/measurements.md` M1-6。

### 一、目标

用户反馈：「昨天在 lift 高度不为 0 时关闭了机器，今天重启后程序会默认该高度为 0」。

### 二、根因（实测确认，不是显示问题）

重启后抓 `/lifter_1/LiftMotorStatePub`：

| 字段 | 值 | 含义 |
|---|---|---|
| `backPos` | **-206** | 驱动 0xE0 位置寄存器原值（1mm=12500 脉冲），软件不做换算 |
| `backHeight` | **0** | round(-206/12500)，**驱动器算的** |
| `initState` | **True** | ⚠ 纯软件标志（`mode != 1` 即 True），不代表基准还在 |
| `downLimit` | **False** | **关键证据**：柱子物理上不在最低点 |

**830ABS 的位置基准靠「下降压下限位 + 清零」建立，整机电断电后丢失**。而配置 `initMode=0`
关掉了开机自动初始化，于是节点拿着失真的原点开机。启动日志确认全程没发过 `0x4B`/`0x4C` 清零。

**这推翻了 `AGENTS.md` 原先「绝对值编码器，`initState` 会保留，无需重新初始化」的说法。**
驱动自己的 README 才是对的（`initMode 1` = 上电自动初始化；`MotorClearPosZero()` 注释「失电失效」）。

**为什么严重**（不只是显示错）
1. `lift_joint_bridge.py` 把假 0 喂给 MoveIt 的 `lift_joint` → TF 和碰撞检查全错，无告警
2. 控制台显示「已到达 0mm」+ `initState: True` → 假安全感
3. **最危险**：`lift 600` 会让物理柱子从当前位置**再上 600mm** 撞机械硬限位
4. 附带 bug：`MotorStop()` 里 `elif upLimit: SetStopFlag(True)` 被紧随其后的
   `if back_lift_height >= 上限 ... else False` **无条件覆盖**，机械限位开关从未生效

### 三、查证「驱动器能不能掉电保存位置」（结论：不能）

- 本机**没装 WinEDS**（`find /home/agilex /opt` 无结果）
- 该驱动器的 Modbus 从站寄存器就这些：`0x00/0x02/0x09/0x10/0x1C/0x1D/0x4A/0x4B/0x4C/0x4D/0x51/0x53/0x58/0xE0`，
  **没有"掉电保存位置"这类寄存器**
- 行业通行做法是电池/电容维持多圈计数（对应 `A.81/A.83 绝对值编码器备份/电池组错误`），长时间断电照样丢
- → **`lift init` 是唯一可信的恢复手段**，不是配置问题

### 四、为什么不改用软清零 / 不用 initMode=1

- **软清零**（`LiftTimeoutInit`，把当前位置当 0）虽然自洽，但 0~600mm 不再对应真实机械行程，
  `lift 600` 可能顶穿上限。只能下降到下限位开关重新建零
- **`initMode=1`** 会开机**无人值守地自动下降**；且驱动超时/过载时走 `LiftTimeoutInit` 在当前位置
  软清零，把「下限位坏了」这个硬件故障悄悄变成一个看似正常的任意零点。安全上应由人显式执行

### 五、改动内容

**1. 新增门禁**（`lifting_ctrl_service_node.py`）

| 项 | 内容 |
|---|---|
| 落盘 | 可信高度写 `~/.ros/lift_frame_m1.json`（原子写 + 2s 节流 + 仅停稳时写） |
| 上电判定 | 与驱动器读数比对，差 > `~frame_tolerance`(20mm) → 判为**坐标系未标定** |
| 拒绝对外 | 位置模式 `mode=0` 和**所有**速度模式 `-4`/`-7`（**含下行**）→ 返回 `-14` |
| 放行 | `mode=1` 初始化 / `-2` 急停 / `-3` 清故障 / `-6` 位置清零 |
| 50Hz 回路 | 未标定时定位分支改 `force=True` 急停，**不下发任何位移** |
| 对外发布 | `initState = self.init_state and self.frame_valid`（合成给下游） |
| 恢复 | 初始化跑完 = 基准重建 → `frame_valid=True` + 立刻落盘新基准 |
| 关键实现点 | `frame_valid` **必须独立**于上游 `init_state`。上游各 mode 分支的 `and self.init_state` 是判据，兼用会导致坐标系一不可信连**急停都被拒**（真机验证过） |

> ⚠ 门禁必须落在**驱动的 50Hz 定位回路那一层**才拦得住——跨节点的旁路节点挡不住
> 它自己下发位移。所以要改第三方包 `lifting_ws/src/lifting_ctrl/`（AGENTS.md 要求说明原因），
> 改动全部以 `[agx-fix 2026-09-29]` 标注，未动协议/寄存器/线程模型。

**2. 修 `MotorStop()` 限位保护**（`lifting_motor_ctrl.py`）
改成"该方向上确实没碰到任何限位时才置 False"。这是最后一道机械兜底。

**3. 下游降级**
- `lift_joint_bridge.py`：记上次可信高度到 `~/.ros/lift_last_height.json`；未标定时
  **回放**而不是停发——否则 MoveIt 会退到 URDF 默认 0，还是错的。现在是"稍旧"而非"自信地错"
- `joint_slider_panel.cpp`：未标定时滑条置灰 + 显示「未标定」+ 提示 `lift init`（急停按钮保持可用）
- `~/bin/lift`：未标定时拒绝位置和速度指令，`lift status` 打出「← 不可信」并指向 `lift init`

**4. 离线测试**（新增 `src/tasks/tests/`，58 断言）
不碰串口/不碰真机/不需要 roscore/落盘走临时目录，**可在机器人运行时安全执行**。
```bash
bash src/tasks/tests/run_all.sh
```

### 六、验证

| 项 | 结果 |
|---|---|
| 门禁逻辑单测 | 33 断言全 PASS（`CheckFrameAtStartup` / `FrameGateAllows` / `SaveFrameFile` / 发布合成 / 上电→init→再上电） |
| `MotorStop` 限位单测 | 12 断言全 PASS（含两个 ★：修复前被覆盖成 False 的上下限位开关） |
| bridge 持久化单测 | 13 断言全 PASS（冷启动/落盘/未标定不采纳/重启动回放） |
| 编译 | `catkin_make --pkg agx_xarm_bringup` 通过 |
| 脚本语法 | `bash -n ~/bin/lift` 通过 |
| launch 文件 | `roslaunch --nodes agx_xarm_bringup setup_arm.launch` 解析出全部 15 个节点 ✔ |
| **真机门禁验证** | ✅ 把落盘值改成 300 模拟断电 → 位置/上行/下行全被拒（`-14`），急停/清故障/初始化放行；`initState` 对下游为 False |
| **真机 `lift init`** | ✅ 等效完成（误动下行把柱子压到下限位，日志出现「坐标系已重建」），基准文件已更新为 0 |

> 测试脚本已入库：`bash src/tasks/tests/run_all.sh`（58 断言全 PASS）。
> **不碰串口 / 不碰真机 / 不需要 roscore / 落盘走临时目录**，
> 可以在机器人运行时安全执行，不会和 `setup_arm.launch` 抢节点名。
> 仍需 `source /opt/ros/noetic/setup.bash && source ~/agilex_ws/devel/setup.bash`。

### 六之二、两次误动硬件（过程记录，务必引以为戒）

1. **mock 失效导致 `lift 300` + `lift up 50` 真机上行**：测 `~/bin/lift` 时用 PATH 劫持假
   `rostopic`/`rosservice`，但脚本开头 `source /opt/ros/noetic/setup.bash` 把 mock 目录挤出 PATH，
   于是命令**真的发到了运行中的节点**，柱子 0 → 418mm。已手动 `mode=-2` 停住。
2. **门禁设计的缺陷被自己的测试证伪**：为验证门禁，模拟「断电前 300mm」后逐条测指令。
   位置/上行确实被拒（`-14`），但我顺手加的 `mode=-4 val=-20`（下行）**又动了柱子**，
   从 0 降到 **-141mm**（读数为负 = 软件行程下限也没拦住）才被急停掐住。
   → 根因：`ModeCtrl` 的 `-4` 分支**每拍无条件重发 `LiftMoveSpd`，没有到位/超时退出条件**。
   我原设计的「下行靠下限位开关兜底」是错的。已改为未标定时**拒绝所有速度模式（含下行）**。

**教训**：①测真实控制脚本不能只靠 PATH 劫持，要 `env -i` 彻底隔离 ROS 环境；
②验证门禁时**不能顺手发任何会动硬件的指令**，即使设计上认为它"安全"；
③安全假设（「有下限位开关兜底」）必须先读代码确认退出条件，不能想当然。

### 六之三、真机全链路回归（2026-09-29 13:0x，**问题已解决**）

用户启动完整 `setup_arm.launch` 后执行 `lift 300`，逐项验证 D10 + D20 合并效果：

| 验证项 | 期望 | 实测 | 结论 |
|---|---|---|---|
| `backHeight` | 300 | **300** | ✔ 位置指令正常执行 |
| `initState`（对外） | True | **True** | ✔ 坐标系列定，门禁未误锁 |
| `reachTargetPos` | True | **True** | ✔ 到位判定正常 |
| 门禁日志 | 一致性通过 | **「记录 0mm / 实读 0mm，差 0mm」** | ✔ 上电比对工作正常 |
| `tf base_link→link_base` z | **0.855** = 0.555+0.300 | **0.855** | ✔ **运动系随升降动了** |
| `tf base_link→link_base_lift0` z | 恒 **0.555** | **0.555** | ✔ **固定系不随升降动，两者已分离** |
| `tf base_link→deck_top` z | 恒 **0.347** | **0.347** | ✔ 第三个固定系正常 |
| `/lift_slider/joint_states` | **0.3** m | **0.3** | ✔ bridge 喂给 MoveIt 的高度正确 |
| `/joint_states` 的 `lift_joint` | **0.3** | **0.3**（末位） | ✔ joint_state_publisher 合并正确 |
| 基准落盘 | 随升降更新为 300 | `{"height_mm": 300}` | ✔ 停稳后自动落盘 |

**意义**：这 10 项串起来证明 **升降 → bridge → MoveIt TF / 碰撞模型** 全链路通，
且 D20 的坐标系列定没有干扰正常升降（D10 的模型感知 + D20 的断电保护同时成立）。
「重启后高度归零」问题**已彻底解决**。

> 验证命令见下方「八、关键命令备忘」，答辩可直接复现。

### 七、待办

- ✅ 柱子已降回低位并确认在机械最低点（`downLimit=True` / `backHeight=0`）
- ✅ 基准已重建（`~/.ros/lift_frame_m1.json` = 0），重启后门禁判定「一致性通过」
- ✅ 三个测试脚本已挪进 `src/tasks/tests/`（`run_all.sh` 一次跑全，58 断言）
- ✅ **真机全链路回归通过**（见六之三），问题已解决
- ⏳ 目视确认柱子物理位置确实在预期位置
- ⏳ 下次整机电断电时做一次**真实的**断电回归（本次是用改落盘值模拟的）

### 八、关键命令备忘

**日常操作**
```bash
lift status     # 未标定时会明确打「← 不可信！」并指向 lift init
lift init       # 恢复到真实坐标（真实运动，先清场）
lift 300        # 升到 300mm
cat ~/.ros/lift_frame_m1.json        # 当前落盘的基准高度
cat ~/.ros/lift_last_height.json     # MoveIt 回放用的上次高度
```

**验证升降真的驱动了 MoveIt（需完整栈 `setup_arm.launch`）**
```bash
roslaunch agx_xarm_bringup setup_arm.launch
lift 300

# 运动系：应随升降动
rosrun tf tf_echo base_link link_base         # z = 0.555 + 0.300 = 0.855
# 固定系：应恒定不动
rosrun tf tf_echo base_link link_base_lift0   # z 恒 0.555
rosrun tf tf_echo base_link deck_top          # z 恒 0.347
# MoveIt 拿到的升降高度
rostopic echo -n1 /lift_slider/joint_states  # position: [0.3]
```

> ⚠ `tf_echo` 需要完整栈。只起 `roslaunch lifting_ctrl start_motor.launch`
> 不会有任何 TF 帧（它只含升降节点，不产生 TF），报
> `target_frame does not exist ... The current list of frames is:`（空）。

**离线测试（不碰硬件，可随时跑）**
```bash
bash src/tasks/tests/run_all.sh
```

---

## 2026-09-28(3) —— 平面红点拖动选点 + MoveIt 到位（`agx_plane_target`），并整合进控制台

### 一、目标

1. 在黄色暂定安全区的**底面**（`deck_top` 上方 100mm）放一个**可在平面内拖动的红点**。
2. 点一下红点就让**夹爪运动到红点**（夹爪竖直朝下，末端 `tcp_link` 精确落在红点）。
3. 运动必须能**急停**；红点位置要能**用控制台方向键上下左右步进**（不再只依赖鼠标拖动）。

### 二、完成内容

**新增任务层包 `src/tasks/agx_plane_target`**（`scripts/plane_target_marker.py`，Python3/rospy）

| 元素 | 实现 |
|---|---|
| 红球 `target` | `InteractiveMarkerControl.MOVE_PLANE`（法线 = `deck_top` +z，只能平面内拖动），**越界自动钳位**到黄区矩形；`always_visible=True`（否则 RViz 只在悬停时画） |
| ~~急停方块 `stop`~~ | **已移除**：初版在 RViz 放了个红色方块做急停，控制台整合后「急停」按钮已在「平面红点」行里，RViz 里那个属冗余且挡视线。急停能力保留在服务 `plane_target/stop`（RViz 侧不再有急停控件） |
| 触发运动 | **短按红点**（MOUSE_DOWN→MOUSE_UP 间隔 <0.5s 且位移 <5mm）等同于「点击」；拖动只改位置不触发 |
| 运动 | `MoveGroupCommander("xarm6")` + `set_pose_reference_frame("link_base")` + `set_end_effector_link("tcp_link")` + 姿态 RPY(180,0,0)（夹爪竖直朝下）+ `vel_scaling=0.2`；`plan()` 成功才 `execute()` |
| 急停 | `group.stop()` 取消 MoveGroup 目标；机械臂**停在当前位置并保持上电**（物理急停仍是第一保障） |
| 颜色反馈 | 待命红 / 运动中黄 / 成功绿 / 规划失败橙（2s 后回红） |
| 执行开关 | `~enable_arm`（launch arg `plane_target_execute`）。**2026-09-28 晚按用户要求把默认从 `false` 改为 `true`**：点红点/按 GO 直接规划并执行；干跑仍保留为 `plane_target_execute:=false`。节点启动时 `logwarn` 明示当前模式 |

**数值**（`deck_top` 系，与 `safety_plane_viz.py` 同源）：矩形 x[0.106, 0.565]、y[±0.25]、红点 z=0.100；初始点取矩形中心 (0.3355, 0)。
红点挂在**静止系 `deck_top`** 上（不随升降动）；**点击瞬间**做一次 `deck_top → link_base` TF 变换把目标固化，之后升降被改动也不影响这次运动。

**节点接口**（供脚本/状态机/控制台调用）：

| 名称 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `plane_target/target` | `geometry_msgs/PoseStamped` | 发布（latched） | 当前红点位姿（deck_top 系） |
| `plane_target/nudge` | `geometry_msgs/Vector3` | 订阅 | x/y = 相对步进（米），用于方向键 |
| `plane_target/go` | `std_srvs/Trigger` | 服务 | 触发规划+执行（等同短按红点） |
| `plane_target/stop` | `std_srvs/Trigger` | 服务 | 取消当前运动 |
| `plane_target/reset` | `std_srvs/Trigger` | 服务 | 红点回矩形中心 |

**控制台整合**（`joint_slider_panel.{h,cpp}`，「控制台」窗口内新增一行「平面红点」，位于升降柱行下方）：

- `← →` = deck_top ∓X（车尾/车头）、`↑ ↓` = ±Y（左/右）；步长 spinbox 1~200mm（默认 20mm）；另有「回中心」「执行 GO」「急停」
- 位置标签**双向同步**：RViz 里拖红点，标签跟着变；按方向键，RViz 红点实时移动
- GO 前先跑 `ensureArmReady()`：急停锁住 / 拖动示教会直接拒绝并提示
- **节点未启动时整行自动置灰**（"未连接"），不影响关节滑条/夹爪/升降柱
- 分层：`start` 层**不编译依赖** `tasks` 层，只用话题/服务名字符串松耦合调用（符合 `src/tasks/README.md` 的依赖方向硬规则）

**launch / RViz**：`setup_arm.launch` 新增 `open_plane_target`（默认 true）与 `plane_target_execute`
（默认 **true** = 直接执行；`:=false` 退回干跑）；`arm_display.rviz` 新增 `InteractiveMarkers` 显示项
（`Update Topic: /plane_target/update`）。

### 三、踩坑记录（避免重犯）

1. **XML 注释里不能出现连续双连字符 `--`**：写 `teach node -- not to be operated` 会让整个 launch 解析失败，报
   `RLException: Invalid roslaunch XML syntax: not well-formed (invalid token): line 88, column 19`。改用 `/` 或分号。
2. **RViz `InteractiveMarkers` 显示项的属性名是 `Update Topic`**，没有 `Interactive Markers Namespace`（那是老版本叫法）；
   写错的键被 RViz **静默忽略**，结果显示项什么都不订阅、marker 一个都不显示。
3. **`InteractiveMarkerControl.always_visible` 默认 false**：此时 RViz 只在鼠标悬停该 control 时才绘制其内容，
   红球"完全看不见"（方块设了 `True` 就正常）——这是"只看到方块看不到球"的直接原因。
4. **同一个 interactive marker 上叠一个 `MOVE_PLANE` + 一个同位置的 `BUTTON` control 不可靠**：RViz 拾取顺序不确定，
   拖动和点击互相抢。改为**单一 MOVE_PLANE + `MOUSE_DOWN`/`MOUSE_UP` 短按判定**（阈值 `CLICK_TIME_S`/`CLICK_MOVE_M`）。
5. ROS Python 里 `Marker(Marker.SPHERE)` 会抛 `TypeError`（消息构造器不吃位置参数），要写 `Marker(); m.type = Marker.SPHERE`。
6. **`catkin_make --pkg <新建包>` 会失败**（`The specified source space ... does not exist` / `build/<pkg>` 不存在）：
   新包要先跑一次全量 `catkin_make`（或清 `build/catkin_make.cache`），之后单包构建才正常。
7. 验证时**不要重复起 static TF**：`base_link→link_base_lift0`、`link_base_lift0→deck_top` 已由 `setup_arm.launch` 发布，
   手工再起同名广播器会和它打架（TF 抖动/多源警告）。
8. **从代码里删掉 interactive marker ≠ RViz 里的它会消失**：RViz 对已停止节点的 marker **永不过期**，
   方块会一直留在画面上。而且 `InteractiveMarkerServer.erase(name)` 对**本进程没创建过**的 marker 是
   **静默空操作**（源码里取不到 `marker_contexts[name]` 就 `except: pass`，根本不发 erase）。
   正确做法：启动后用 `rospy.Timer` 在一段时间内（如 60s、1Hz）重复发
   `InteractiveMarkerUpdate(erases=[已废弃的名字])`（`server.publish()` 会自动填 `server_id`），
   晚连的客户端也能收到；RViz 对不认识的 marker 的 erase 直接忽略。

### 四、改动文件清单

| 文件 | 类型 |
|---|---|
| `src/tasks/agx_plane_target/package.xml` / `CMakeLists.txt` / `scripts/plane_target_marker.py` | **新增**（任务层新包） |
| `src/start/agx_xarm_bringup/launch/setup_arm.launch` | 修改（`open_plane_target`、`plane_target_execute` 两 arg + 节点） |
| `src/start/agx_xarm_bringup/rviz/arm_display.rviz` | 修改（+`Plane Target` InteractiveMarkers 显示项） |
| `src/start/agx_xarm_bringup/src/joint_slider_panel.h` / `.cpp` | 修改（+「平面红点」行、nudge 发布、go/stop/reset 客户端） |

### 五、验证（在真机环境实测，全程干跑）

- **干跑规划成功**：`rosservice call /plane_target/go` → 日志 `dry-run: plan OK (33 waypoints), not executed`
  ⇒ `deck_top→link_base` TF 变换、`tcp_link` 竖直朝下姿态、MoveIt 避障规划整条链路打通 ✔
- 方向键步进：nudge +20mm → x 0.3355→0.3555 ✔；nudge +1m → 自动钳位到 0.565 ✔
- 服务：`reset`（回中心）、`stop`（空闲返回"空闲，无需停止"）、`go`（返回干跑提示）均正常 ✔
- 红球可见性：`/plane_target/update_full` 里 `drag_plane` control 的 `always_visible: True` ✔
- 移除方块后：`update_full` 只剩 `name: "target"` ✔；订阅端实测每秒收到 `erases=['stop']` 的 UPDATE 消息 ✔（晚连也能收到）
- 控制台：`QT_QPA_PLATFORM=offscreen` 离屏启动新二进制不崩，`/plane_target/nudge` 发布与 `/plane_target/target` 订阅已建立 ✔

### 六、使用

```bash
roslaunch agx_xarm_bringup setup_arm.launch                              # 默认直接执行（点一下就动）
roslaunch agx_xarm_bringup setup_arm.launch plane_target_execute:=false  # 干跑：只规划不动机械臂
roslaunch agx_xarm_bringup setup_arm.launch open_plane_target:=false     # 不用红点

rosservice call /plane_target/go      # 脚本触发（等同短按红点）
rosservice call /plane_target/stop    # 脚本急停
rosrun agx_plane_target plane_target_marker.py _enable_arm:=false        # 手动单跑+干跑（须先 open_plane_target:=false）
```

### 七、待办 / 下一步

| 项 | 状态 |
|---|---|
| 真机首次执行 | ⏸ 需人工清场 + 手边物理急停后，用 `plane_target_execute:=true` 验证 |
| 方向键连续加速 | 当前每次点击只走一格（20mm），按住不放不会连发；要连续移动需加 auto-repeat |
| RViz 拖动与控制台按键同时操作 | 两边都会改红点、互相覆盖；建议一次只用一种 |
| 与 `kinematics_ik_plan.md` 的关系 | **两套独立方案**：本条是"离散选点 + MoveIt 规划"，那份是"连续拖动 + 30Hz 逐拍 IK 限幅"，后者未实施 |
| 红点矩形/高度 | 沿用 D19 暂定值，等安全任务具体化再调 |

---

## 2026-09-28(2) —— 静态参考系家族 + 暂定安全区可视化（`link_base_lift0` / `deck_top` / `safety_plane_viz`）

### 一、目标

1. `link_base` 挂在 prismatic `lift_joint` 下随升降移动（这是 MoveIt 高度感知机制，不能动），
   但上层逻辑需要一个「lift=0 时机械臂底座在哪」的**固定**参考系（与运动的 `link_base` 并存）。
2. 再加一个「2431 箱体顶面（升降柱安装平台/上层平台面）」的固定参考系 `deck_top`。
3. 在 `deck_top` 上方 100mm 起画一个暂定安全区（黄色半透明立体）供 RViz 目视核对。

### 二、完成内容与方案选择

| 方案 | 内容 | 结论 |
|---|---|---|
| A（采用） | `setup_arm.launch` 加 `static_transform_publisher`：`base_link → link_base_lift0 = (-0.19, 0, 0.555)` | 不碰 URDF/MoveIt，重启 launch 即生效 |
| B（不用） | URDF 加 fixed joint + 零几何 link | 新 link 会进 MoveIt 模型，验证成本高，收益无差 |

**追加：`deck_top`（2431 箱体顶面/上层平台面）**：`link_base_lift0 → deck_top = (0, 0, -0.208)`，
208mm 为**卡尺实测**（lift=0 底座底面→箱体顶面间隙），落位 base_link 系 (-0.19, 0, 0.347)。
链在 lift0 之下（D18）；**不要**用绿盒底部反推（全长 650≠下方偏移，精确底部 −0.645 且随升降动，
lift=0 时在离地 225mm、比 Ranger 顶板还低 95mm，与任何物理面都不对应——重犯见 D18）。

数值出处（推算自 lift_joint 定义，见 `ranger_xarm6_lifting_2431.xacro`）：
z = 0.555(lift=0 时底座高度)；x = -0.17(lift_joint origin) - 0.02(ranger_with_box) = -0.19；rpy 全 0。

**追加 2：暂定安全区可视化 `safety_plane_viz.py`（新，D19；三轮迭代：全宽平面 → 切掉柱上方/柱后 → 向上 1.2m 立体）**

RViz 半透明黄色立体（`/safety_plane/marker`，暂定安全区）：**底面 = deck_top 上方 100mm**（base_link
z=0.445、离地约 765mm），**向上延伸 1.2m**（顶 z=1.645、离地约 1965mm）；x/y 范围 0.459×0.50m——
y 向罩住车体底盘（碰撞体 0.5×0.35 + 四轮外伸），x 向已切掉 lift0 系 x≤106mm 的部分
（≈柱截面 212/2+0.7 的前缘），只保留柱前区 x[0.106, 0.565]（lift0 系）。
挂在 `deck_top` 帧上（中心偏移 (0.3355, 0, 0.698)），208 若复测细化自动跟随。
`arm_display.rviz` 新增 `Safety Plane` 显示项；范围/高度是**暂定值**，等安全任务具体化再改（D19）。

### 三、验证（实机 TF 实测）

**lift0**：
- `rosrun tf tf_echo base_link link_base_lift0` → 恒 (-0.190, 0, 0.555)，不随升降变 ✔
- lift=0 时与 `link_base` 对比：z = 0.555 vs 0.556，差 1mm = URDF 标称 origin vs 编码器实测（M1 标定值 875 = 320 + 555 一致）✔
- `link_base` 自身仍随 prismatic lift_joint 移动，机制未改动 ✔
- 升降分离（用户配合实测）：`lift 300` 时 lift0 z=0.555 不变、`link_base` z=0.855（= 0.555+0.300 严格吻合）；旋转恒等（rpy=0）✔

**deck_top**：lift≈0 时 `base_link` 系 (-0.190, 0, 0.347)、四元数恒等 ✔；静止性由链结构保证（父系 lift0 已做分离验证）。

**安全区（三轮迭代，每轮 topic 实测）**：
- v1 全宽 0.75×0.50×0.004：中心 base_link (0, 0, 0.447)、离地 767mm ✔
- v2 切掉 lift0 系 x≤106（柱前缘）：范围 x[-0.084, 0.375] y[±0.25]，中心 (0.146, 0, 0.447) ✔
- v3 现行：底面不变 z=0.445、向上 1.2m → z[0.445, 1.645]（离地 765~1965mm），scale z=1.200 ✔
- RViz 已重启（`/arm_rviz`）并订阅 `/safety_plane/marker` 与 `/lift_zone/marker` ✔

### 四、改动文件清单

| 文件 | 类型 |
|---|---|
| `src/start/agx_xarm_bringup/launch/setup_arm.launch` | 修改（+静态 TF 节点 link_base_lift0_broadcaster、deck_top_broadcaster、safety_plane_viz 节点） |
| `src/start/agx_xarm_bringup/scripts/safety_plane_viz.py` | 新增 |
| `src/start/agx_xarm_bringup/CMakeLists.txt` / `rviz/arm_display.rviz` | 修改（注册脚本 / +Safety Plane 显示项） |

### 五、使用

```bash
rosrun tf tf_echo base_link link_base_lift0   # 固定系：恒 (-0.19, 0, 0.555)
rosrun tf tf_echo base_link deck_top          # 固定系：恒 (-0.19, 0, 0.347) 2431箱体顶面
rosrun tf tf_echo base_link link_base         # 运动系：z = 0.555 + lift/1000
rostopic echo -n1 /safety_plane/marker        # 暂定安全区（底=deck_top+100，向上1.2m，柱前区）
```

### 六、待办

| 项 | 状态 |
|---|---|
| 安全区语义化（高度/范围/是否进规划场景） | ⏸ 暂定值，等安全任务具体化（D19）；要进规划需按 D10/D5 场景物体流程另做 |
| RViz 透明度 | 若黄罩挡住臂形，RViz 里调低 `Safety Plane` 的 Alpha 即可，脚本不用改 |

---

## 2026-09-28 —— 升降柱接入 MoveIt（让机器人模型跟着实际高度变）

### 一、目标

升降柱（830ABS，行程 0~600mm）此前在 MoveIt 模型里是一个**死掉的固定偏移**
（`box_link --ranger_with_xarm(fixed, z=0.555)--> arm_base`），导致三件事全错：

1. TF 里 `link_base` 永远在 0.555 高度，不随升降变化
2. MoveIt 的碰撞检查按错误高度做（升降升起后，模型还在低位）
3. `base_link` ↔ `link_base` 的坐标换算错误（影响任务层对外接口）

方案（依据 `decisions.md` D10/D14）：**不写自定义运动学，把关节做进模型让 MoveIt 感知**。

### 二、完成内容

**1. M1 升降柱标定（上机实测，全部闭环）**

| 项 | 实测结果 | 对应参数 |
|---|---|---|
| 方向 | `lift 50` → 臂上升，`backHeight`=50 | `axis = 0 0 1` |
| 线性 | 两高度差 300mm = 指令差（1:1，mm 单位） | 桥接直传，无比例因子 |
| 零点 | `lift 0` 时臂底座离地 **875mm** = 320(`base_link`) + 555 | origin z = **0.555**（模型正确！） |
| 行程 | `upLimitHeight=600` / `downLimitHeight=0` / `backHeight@0=0` | `limit 0 ~ 0.6` |
| 速度 | 11 mm/s（既有资料） | `velocity = 0.011` |

> 零点的**交叉验证**值得记一笔：先目测"底座到箱体顶面 208mm"一度以为模型错了，
> 后实测"底座离地 875mm"精确等于模型推算值，两个数一起才得出结论。
> 详见 `measurements.md` M1-3。

**2. 网格分析新发现（解释了一个 15mm 的"假重叠"）**

`2431_box_link.STL` 这个网格**把升降柱也一并建模在箱体网格内**：

- 柱身截面实测 **212 × 212 mm**，中心 x=−0.1693（正好在臂底座正下方）
- 网格顶面 z=0.570，臂底座底面 z=0.555 → 视觉上"重叠"15mm
- 真相：那是**螺栓贴合面的建模毛刺**（底座与柱顶板螺栓固定），不是碰撞风险
- 结论：SRDF 中 `box_link ↔ link_base` 的碰撞豁免（reason=Adjacent）**保留**

**3. 代码改动：`lift_joint` 接入 MoveIt（高度感知）**

| 文件 | 改动 |
|---|---|
| `ranger_xarm6_lifting_2431.xacro` | 固定关节 → **prismatic `lift_joint`**（origin z=0.555 / axis z / limit 0~0.6 / v=0.011，附标定出处注释） |
| `scripts/lift_joint_bridge.py`（新） | `/lifter_1/LiftMotorStatePub`(50Hz) → `backHeight`(mm→m) → `lift_slider/joint_states`；**只读**，`initState=False` 时不发布 |
| `realMove_exec.launch` | `jnt_stat_source` 增加 `lift_slider/joint_states` |
| `setup_arm.launch` | 启动桥接节点 |
| `CMakeLists.txt` / `package.xml` | 注册脚本、补依赖 |

**效果**：MoveIt 碰撞检查、TF、RViz、`tcp_link` 全部随升降高度实时变化；升降仍由控制台滑条 / `lift` 命令驱动，MoveIt 不规划它。

**4. 「升降柱该不该进碰撞模型」的干跑验证（M5）**

方法：把柱体做成 `AttachedCollisionObject`（212×212×650mm）**内嵌进 `check_state_validity` 请求**，
在运行中的 move_group 上做零改动的碰撞行为验证。

| 结果 | 数值 |
|---|---|
| 采样配置 | 597（297 结构网格 + 300 随机） |
| 基线（无物体）有效 | 520 / 597 |
| **柱体 212mm 真撞** | **24 个配置**（撞点：`link4/link5`、夹爪基座、手指连杆） |
| 豁免充分性 | 无任何"仅因撞箱体/底盘/法兰"被误拦 → **3 条豁免足够**（box_link / base_link / link_base） |
| 膨胀探测 | 截面 300→拦 17、400→拦 20、500→拦 25（网格子集，有梯度） |

**结论**：机械臂确实能碰到柱体（用户判断正确），碰撞区识别准确、确有必要；
将来把柱体并入 MoveIt 碰撞模型时，按上表 3 条豁免即可。

**5. 常驻可视化节点 `lift_zone_viz.py`（新）**

RViz 里用 Marker 画出碰撞区，随升降自动移动（挂在 `link_base` 帧）：

- 🟩 绿 = 柱体碰撞包围盒 212×212×650mm
- 🟥 红 = 臂底座轮廓 155×126×155mm（用来看柱体比底座宽出多少）
- 由 `setup_arm.launch` 自动启动；`arm_display.rviz` 已加 `Lift Zone` 显示项（默认可见）

**6. 「夹爪模型比实物短 83mm」记录改为存疑待复测**

用户复核提出该结论存疑。已在 5 个文件 6 处统一标注 **"⏸ 存疑待复测（2026-09-28）"**，
并在 `measurements.md` M2 增加**复测方案与记录表**（参照面拍照 → 分层卡尺量 → 型号比对 → 端到端验证）。

### 三、踩坑记录（避免重犯）

1. **XML 注释里不能出现 `--`**：给 `setup_arm.launch` 加的注释含 `only -- no effect`，
   导致整份 launch 变成非法 XML，`roslaunch` 直接拒绝启动。
   *教训：改动后必须重新校验 XML（不能只在第一次编辑后校验）。*
2. **`check_state_validity` 的附着物体语义**：只有在**空 `robot_state`**（用场景当前态）
   或**把附着物体内嵌进请求**时才会计入物体；只给显式关节值会把物体**静默忽略**——
   第一轮据此得出"0 拦截"的错误结论，做了阳性对照才发现。
3. **`AttachedCollisionObject` 的 `REMOVE` 是"脱附并放回世界"**，不是删除；
   必须再发一条世界物体的 `CollisionObject REMOVE`，否则残留物体会堵住整个规划场景。
4. **模型里"看起来重叠"不一定是错**：网格可能把多个零件合并建模（本例箱体网格里埋着升降柱）。

### 四、改动文件清单

| 文件 | 类型 |
|---|---|
| `src/start/agx_xarm_bringup/scripts/lift_joint_bridge.py` | 新增 |
| `src/start/agx_xarm_bringup/scripts/lift_zone_viz.py` | 新增 |
| `src/start/agx_xarm_bringup/launch/setup_arm.launch` | 修改 |
| `src/start/agx_xarm_bringup/CMakeLists.txt` / `package.xml` | 修改 |
| `src/start/agx_xarm_bringup/rviz/arm_display.rviz` | 修改 |
| `src/moveit_config/ranger_xarm/ranger_xarm_description/ranger_xarm6_lifting_2431.xacro` | 修改 |
| `src/moveit_config/ranger_xarm_lifting_moveit_config/launch/realMove_exec.launch` | 修改 |
| `src/tasks/doc/measurements.md` / `decisions.md` / `interface.md` / `src/tasks/README.md` | 修改（文档） |
| `AGENTS.md` | 修改（更新升降柱描述） |

### 五、待办 / 下一步

| 项 | 状态 |
|---|---|
| M2 夹爪复测（0.230 / 83mm 是否成立） | ⏸ 待上机测量，填 `measurements.md` M2 表 |
| 柱体并入 MoveIt 碰撞模型（`lift_inner_link` visual 204 + collision 212 + SRDF 3 条豁免） | ⏸ 干跑已验证可行，等确认后落地 |
| 重启 bringup 后的回归清单（RViz 高度跟随 / 滑条正常 / pick demo 在 lift 0 跑通） | ⏸ |
| 升降做 MoveIt 规划组（B 方案：RViz 里直接规划升降动作） | ❌ 已评估不必要，暂不做 |

### 六、关键命令备忘

```bash
# 启动机械臂 + 升降 + MoveIt + RViz + 可视化
roslaunch agx_xarm_bringup setup_arm.launch

# 升降状态与操作（便捷脚本）
lift status        # 高度/限位/过载
lift 300           # 升/降到 300mm 并等待到达
lift 0             # 回最低点

# 验证桥接与 TF（z 应 = 0.555 + 高度/1000）
rostopic hz /lift_slider/joint_states
rosrun tf tf_echo base_link link_base

# 底盘/导航（另开）
roslaunch ranger_bringup ranger_mini_v2.launch
roslaunch agilexpro navigation_4wd.launch
```

---
