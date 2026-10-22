# 任务层接口契约

> 本文件是**冻结版**接口约定。写代码只许按这份来；要改接口，先改本文件并说明原因。
>
> 状态标记：✅ 已定 ｜ ⏸ 待实测/待实现 ｜ ❌ 明确不做

---

## 0. 包布局

```
src/tasks/
├── agx_task_msgs/                  # 消息 + 服务 + action（无逻辑）
├── agx_task_core/                  # 基类 + 能力封装 + 注册表 + Runner（不含具体任务）
└── agx_task_arm_move_to_pose/      # 具体任务，一个任务一个包
```

`agx_task_bringup` 负责把所有任务库链接进同一个 runner 可执行文件（静态注册清单）。

---

## 1. 消息定义

```text
# agx_task_msgs/msg/TaskState.msg
uint8 IDLE      = 0
uint8 RUNNING   = 1
uint8 SUCCEEDED = 2
uint8 FAILED    = 3
uint8 CANCELED  = 4

string  task_id        # 本次执行的唯一 id（时间戳生成）
string  task_name
uint8   state
string  stage          # 人可读，如 "移动到目标位姿"
float32 progress       # 0~1，-1 = 未知
string  message        # 失败原因 / 结果摘要
```

```text
# agx_task_msgs/action/RunTask.action
string task_name
string params_yaml     # 可选，覆盖任务默认参数；空 = 只用 YAML
---
bool   success
string message
string result_yaml     # 可选产出，第一版留空
---
agx_task_msgs/TaskState state
```

```text
# agx_task_msgs/srv/ListTasks.srv
---
string[] task_names
string[] descriptions
```

| 接口 | 名称 |
|---|---|
| action server | `/agx_task/run` |
| latched 话题 | `/agx_task/state`（同 `TaskState`，给 UI/监控看，不需要 action 客户端） |
| service | `/agx_task/list` |

✅ 已定

---

## 2. TaskBase

```cpp
// agx_task_core/include/agx_task_core/task_base.hpp
namespace agx_task {

enum class TaskResult { Succeeded, Failed, Canceled };

class TaskContext;

class TaskBase {
public:
  using Ptr = std::shared_ptr<TaskBase>;
  virtual ~TaskBase() = default;

  virtual std::string name() const = 0;                 // 唯一 id，与注册名一致
  virtual std::string description() const { return ""; }

  // 每次执行前调用一次。返回 false → Runner 直接判失败，不进入 run()
  virtual bool configure(TaskContext& ctx) = 0;

  // 阻塞执行。收到 cancel 后应尽快返回 Canceled
  virtual TaskResult run(TaskContext& ctx) = 0;

  // 可能从其它线程调用；默认空实现
  virtual void cancel() {}
};

}  // namespace agx_task
```

✅ 已定

---

## 3. TaskContext

```cpp
class TaskContext {
public:
  ros::NodeHandle& nh();                 // 全局
  ros::NodeHandle& pnh();                // 任务参数：/agx_task/<task_name>/*

  ArmCapability&     arm();
  GripperCapability& gripper();
  LiftCapability&    lift();
  NavCapability&     nav();

  // 反馈（线程安全，映射到 action feedback + /agx_task/state）
  void setStage(const std::string& stage);
  void setProgress(float progress);      // 0~1，-1 = 未知

  // 取消
  bool canceled() const;                 // 轮询
  bool sleep(double seconds);            // 可被 cancel 打断；返回 false = 被取消

  // 日志（自动加 [task_name] 前缀）
  void log(const std::string& msg);
  void logWarn(const std::string& msg);
  void logError(const std::string& msg);
};
```

**参数优先级**：`config/<task>.yaml` 是默认值；`params_yaml` 非空时，Runner 用 yaml-cpp（系统已装 0.6.2）解析后覆盖 `/agx_task/<task_name>/` 下的同名参数，然后才调 `configure()`。任务代码只读 `pnh()`，不关心来源。

✅ 已定

---

## 4. 四个 Capability

能力对象由 `TaskContext` 构造并持有取消标志，因此所有阻塞调用都能被 cancel 打断。

### 4.1 ArmCapability

```cpp
class ArmCapability {
public:
  // 就绪与状态
  bool waitReady(double timeout_s = 10.0);      // move_group 可用 + 机械臂无错误
  bool isMoving() const;
  void stop();

  // 目标位姿：位置 + 朝向都由调用方显式给出
  // frame 默认 "base_link"；也接受 "world"（需导航栈在跑）
  // 参考点 = TCP（tcp_link，指尖下沿）
  bool moveToPose(const geometry_msgs::Pose& target,
                  const std::string& frame = "base_link",
                  double timeout_s = 20.0);

  // 直线运动，用于接近/撤离
  bool moveLinear(const std::vector<geometry_msgs::Pose>& waypoints,
                  const std::string& frame = "base_link",
                  double eef_step = 0.01);

  // SRDF 命名位姿："initial" / "home"
  bool moveToNamed(const std::string& name);

  // 保留给抓放类任务，本任务不用
  bool pick();
  bool place();
};
```

**实现要点**

- **TCP 参考点**：SRDF 规划组 `xarm6` 的 `tip_link` 是法兰 `link6`。要用夹爪尖端，需在 URDF 增加固定的 `tcp_link`，然后用 `move_group.setEndEffectorLink("tcp_link")`。
  `tcp_link` 定义：从 `link6` 固定偏移 `xyz = (0, 0, 0.170)`，即**夹爪闭合状态下两指尖下沿的中点**（**实机测量值，2026-09-29 复测确认**：闭合 0.170 m；张开值未复测）。
  ✅ 此前"URDF 夹爪网格比实测短约 0.083 m、碰撞体偏短"的推断**已作废**：网格 FK 0.147 m 与实测只差 23 mm，且方向保守。详见 `measurements.md` 的 M2、决策 `decisions.md` D12/D16。
- **朝向"末端朝下"的四元数不写死在代码里**，放进任务 YAML（`target_qx/qy/qz/qw`），上机器标定一次。接口只收四元数。
- 实现复用现有 `agx_xarm_pick` 里的 MoveIt 用法（`MoveToA` / `MovePath`），去掉其中的硬编码。

✅ 接口已定 ｜ ⏸ TCP 与朝向待标定

### 4.2 LiftCapability

```cpp
class LiftCapability {
public:
  bool  isConnected() const;
  int   height() const;                          // mm（backHeight）
  bool  isInitialized() const;                   // initState
  bool  isFault() const;                         // overflowI/V, overLoad, underVol, encodeErr
  bool  isMoving() const;

  bool  waitSettled(double timeout_s = 80.0);    // reachTargetPos && 速度≈0；任务开头必调
  bool  isInRange(int min_mm, int max_mm) const; // 含故障检查

  bool  moveTo(int height_mm, double timeout_s = 80.0);  // mode=0，内部轮询到到达
  bool  moveBy(int delta_mm, double timeout_s = 80.0);
  bool  setVelocity(int mm_per_s);               // mode=-4，正=上升，不等待
  bool  setSpeed(int rpm);                       // mode=-5
  bool  stop();                                  // mode=-2
  bool  clearError();                            // mode=-3

  // 危险：会真实下降到下限位
  bool  home();                                  // mode=1
};
```

- 后端：服务 `/lifter_1/LiftingMotorService`（`bt_task_msgs::LiftMotorSrv`），状态话题 `/lifter_1/LiftMotorStatePub`
- 服务**非阻塞**（`config/1_lifting_motor_config.json` 里 `"callBackMode": 1` → `SververCallbackImmediately`），所以 `moveTo` 必须内部轮询状态话题直到 `reachTargetPos`，超时返回 false
- 默认超时 80s（全程 600mm 约 55s + 余量）
- 危险操作（`home()`）单独一组，代替 confirm 参数

✅ 已定

### 4.3 GripperCapability

```cpp
class GripperCapability {
public:
  bool  isConnected() const;
  bool  isInitialized() const;
  bool  isMoving() const;
  int   gripState() const;                       // ⏸ 0/1/2 含义待实测
  float position() const;                        // 0~1000

  bool  open(float force = 50.0);                // position = 1000
  bool  close(float force = 50.0);               // position = 0
  bool  moveTo(float position, float force, float speed = 0.0);

  bool  waitIdle(double timeout_s = 3.0);
  bool  waitGrasp(double timeout_s = 3.0);       // ⏸ 依赖 grip_state 语义
};
```

- 后端：发 `/gripper/ctrl`（`dh_gripper_msgs::GripperCtrl`），收 `/gripper/states`（**复数**）
- 注意：`agx_xarm_pick` 里订阅的是 `/gripper/state`（单数、带尾空格、整行被注释），是错的，不要照抄

✅ 接口已定 ｜ ⏸ 两个状态语义待实测

### 4.4 NavCapability

```cpp
class NavCapability {
public:
  bool waitForServer(double timeout_s = 5.0);
  bool moveTo(const geometry_msgs::PoseStamped& goal, double timeout_s = 120.0);
  bool cancel();
  geometry_msgs::PoseStamped currentPose() const;   // frame 从参数读，默认 "world"
};
```

- 后端用 `move_base` 的 **action**（`move_base_msgs::MoveBaseAction`）
- **不要**用 `/move_base/result` 那个话题（现有 `order.cpp` 就是伪造它来假装导航到达）
- 第一版只定接口，不实现

✅ 接口已定 ｜ ⏸ 未实现

---

## 5. 注册表与 Runner

```cpp
class TaskRegistry {
public:
  using Factory = std::function<TaskBase::Ptr()>;
  static TaskRegistry& instance();
  void add(const std::string& name, Factory factory);
  TaskBase::Ptr create(const std::string& name) const;
  std::vector<std::string> names() const;
};

#define AGX_REGISTER_TASK(NAME, CLASS)                                    \
  namespace {                                                             \
  const bool g_agx_task_reg_##CLASS =                                     \
      agx_task::TaskRegistry::instance().add(                             \
          NAME, []() { return agx_task::TaskBase::Ptr(new CLASS()); });   \
  }
```

**Runner（`agx_task_runner`，单节点）行为**

| 情况 | 行为 |
|---|---|
| 空闲 + 收到 goal | accept → configure → 起 worker 线程跑 run → 发 result |
| 忙 + 收到 goal | reject，message 说明"正在执行 xxx" |
| 执行中收到 cancel | 调 `task->cancel()`，等 run 返回 Canceled → `setPreempted` |
| 任何时刻 | 每 0.5s 把 TaskState 发到 `/agx_task/state`（latched） |
| 任务名不存在 | reject，附带 `/agx_task/list` 里的可用列表 |

✅ 已定

---

## 6. 跨接口约定

1. **坐标系**：对外默认 `base_link`（底盘，不随升降动）；导航用 `world`；`link_base` 只在实现内部出现，不出现在任务 YAML 里。需要"机械臂底座固定安装位"参考时用 `link_base_lift0`（静态 TF，lift=0 快照，`base_link` 系恒 (-0.19, 0, 0.555)，见 `decisions.md` D17），运动的 `link_base` 与其并存
2. **单位**：长度 m（ROS 惯例）；升降柱对外 mm（跟随现有服务）；夹爪 0~1000（跟随现有驱动）
3. **超时**：所有阻塞调用必须带 timeout，禁止无限等待
4. **失败**：返回 `false`，不抛异常；由任务决定是失败还是重试
5. **取消**：能力对象持有取消标志，长调用可被 cancel 打断
6. **日志**：只走 `ctx.log*`，自动带任务名前缀
7. **参数**：默认值在任务包 YAML，覆盖走 `params_yaml`
8. **并发**：串行，一次一个任务。硬件只有一台，多任务并行会打架
9. **升降柱**：由上层串行安排（另一个任务负责时，它结束本任务才开始）。本任务开头 `waitSettled()` + `isInRange()`，**不做锁**

---

## 7. 示例任务：`arm_move_to_pose`

任务名 `arm_move_to_pose`，包 `agx_task_arm_move_to_pose`，类 `ArmMoveToPoseTask`。

场景：机械臂从当前位姿（由另一个任务摆好）移动到 `base_link` 下的指定位姿，末端朝下。

```cpp
class ArmMoveToPoseTask : public TaskBase {
public:
  std::string name() const override { return "arm_move_to_pose"; }
  std::string description() const override { return "机械臂移动到指定位姿（末端朝下）"; }

  bool configure(TaskContext& ctx) override {
    auto& p = ctx.pnh();
    p.param("frame",          frame_, std::string("base_link"));
    p.param("target_x",       x_, 0.35);
    p.param("target_y",       y_, 0.00);
    p.param("target_z",       z_, 0.20);
    p.param("target_qx",      qx_, 1.0);      // 末端朝下，上机器标定
    p.param("target_qy",      qy_, 0.0);
    p.param("target_qz",      qz_, 0.0);
    p.param("target_qw",      qw_, 0.0);
    p.param("lift_min_mm",    lift_min_, 0);
    p.param("lift_max_mm",    lift_max_, 50);
    p.param("move_timeout_s", timeout_, 20.0);
    return true;
  }

  TaskResult run(TaskContext& ctx) override {
    ctx.setStage("等待升降柱稳定");
    if (!ctx.lift().waitSettled(80.0)) {
      ctx.logError("升降柱未稳定");
      return TaskResult::Failed;
    }
    if (!ctx.lift().isInRange(lift_min_, lift_max_)) {
      ctx.logError("升降柱高度超出允许范围");
      return TaskResult::Failed;
    }

    ctx.setStage("移动到目标位姿");
    geometry_msgs::Pose target;
    target.position.x = x_;  target.position.y = y_;  target.position.z = z_;
    target.orientation.x = qx_; target.orientation.y = qy_;
    target.orientation.z = qz_; target.orientation.w = qw_;
    if (!ctx.arm().moveToPose(target, frame_, timeout_)) {
      ctx.logError("规划或执行失败");
      return TaskResult::Failed;
    }

    ctx.setStage("完成");
    return TaskResult::Succeeded;
  }

private:
  std::string frame_;
  double x_{0.35}, y_{0.0}, z_{0.2};
  double qx_{1.0}, qy_{0.0}, qz_{0.0}, qw_{0.0};
  int    lift_min_{0}, lift_max_{50};
  double timeout_{20.0};
};

AGX_REGISTER_TASK("arm_move_to_pose", ArmMoveToPoseTask)
```

**注意**：本任务**不调用** `moveToNamed("initial")`。初始位姿由另一个任务摆好，本任务直接从"当前状态"规划——`moveToPose` 天然如此，不需要知道上一动是谁做的。

---

## 8. 明确不做（第一版）

- pluginlib 动态加载任务（先用静态注册 + `agx_task_bringup` 链接）
- 多任务并行 / 任务队列
- `result_yaml` 的实际内容（先留空）
- `NavCapability` 的实现（接口先定）
- 夹爪 `waitGrasp` 的实际启用（等 `grip_state` 实测）

---

## 9. 待实测清单（阻塞实现的部分）

| 项 | 用途 | 记录位置 |
|---|---|---|
| 升降柱方向 / 比例 / 零点 / 行程 | 插入棱柱关节 + 桥接节点 | `measurements.md` |
| ~~`tcp_link` 偏移~~ | 已实测：闭合 **0.170 m**（2026-09-29 复测确认，已写入 URDF） | `measurements.md` M2 |
| ⏸ 夹爪模型"短 0.083 m"**存疑待复测**（2026-09-28） | 若成立则影响碰撞检查，见 `decisions.md` D16 | `measurements.md` M2 复测方案 |
| 目标平面示教读数（2~3 点） | 任务 YAML 的 `target_z` + 四元数 | `measurements.md` |
| `grip_state` 0/1/2 语义 | GripperCapability（本任务不阻塞） | `measurements.md` |
