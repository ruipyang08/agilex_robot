#include "joint_slider_panel.h"

#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <urdf/model.h>
#include <xarm_msgs/ClearErr.h>
#include <xarm_msgs/RobotMsg.h>
#include <xarm_msgs/SetInt16.h>

#include <QApplication>

#include <QHBoxLayout>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace agx_xarm_bringup
{
const std::array<std::string, 6> JointSliderWindow::kJointNames = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

namespace
{
double rad2deg(double rad)
{
  return rad * 180.0 / M_PI;
}

int radToSlider(double rad)
{
  return static_cast<int>(std::lround(rad2deg(rad) * 10.0));
}

// MoveIt named pose "initial": joint5 = -90 deg, other arm joints 0.
// Gripper initial is open (DH position 1000).
const std::array<double, 6> kInitialArm = {0.0, 0.0, 0.0, 0.0, -1.5708, 0.0};
const int kInitialGripper = 1000;
}  // namespace

JointSliderWindow::JointSliderWindow(QWidget* parent)
  : QWidget(parent)
  , timer_(new QTimer(this))
  , speed_slider_(nullptr)
  , status_label_(new QLabel("等待关节状态"))
  , have_state_(false)
  , synced_(false)
  , gripper_synced_(false)
  , driving_(false)
  , pending_(false)
  , pending_gripper_(false)
  , updating_(false)
  , teaching_(false)
  , have_arm_state_(false)
  , arm_state_(4)
  , arm_mode_(0)
  , arm_err_(0)
  , gripper_position_(1000.0f)
  , lift_connected_(false)
  , lift_dragging_(false)
  , lift_synced_(false)
  , pending_lift_(false)
  , lift_height_(0)
  , lift_min_(0)
  , lift_max_(600)
  , lift_reached_(true)
  , lift_initialized_(false)
  , lift_fault_(false)
  , lift_last_sent_(-1)
  , plane_connected_(false)
  , plane_have_(false)
  , plane_x_(0.0)
  , plane_y_(0.0)
  , grasp_connected_(false)
  , grasp_busy_(false)
  , grasp_state_("idle")
{
  for (int i = 0; i < 6; ++i)
  {
    lower_[i] = -M_PI;
    upper_[i] = M_PI;
    current_[i] = 0.0;
  }

  QVBoxLayout* layout = new QVBoxLayout(this);
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);

  QPushButton* home_button = new QPushButton("回到初始位置");
  layout->addWidget(home_button);
  connect(home_button, &QPushButton::clicked, this, &JointSliderWindow::goToInitialPose);

  QHBoxLayout* teach_row = new QHBoxLayout();
  QPushButton* teach_button = new QPushButton("开始拖动示教");
  QPushButton* teach_stop_button = new QPushButton("结束拖动示教");
  teach_row->addWidget(teach_button);
  teach_row->addWidget(teach_stop_button);
  layout->addLayout(teach_row);
  connect(teach_button, &QPushButton::clicked, this, &JointSliderWindow::startTeach);
  connect(teach_stop_button, &QPushButton::clicked, this, &JointSliderWindow::stopTeach);

  QHBoxLayout* speed_row = new QHBoxLayout();
  speed_row->addWidget(new QLabel("速度"));
  speed_slider_ = new QSlider(Qt::Horizontal);
  speed_slider_->setRange(10, 100);
  speed_slider_->setValue(30);
  speed_row->addWidget(speed_slider_);
  layout->addLayout(speed_row);

  QHBoxLayout* gripper_row = new QHBoxLayout();
  QLabel* gripper_name = new QLabel("夹爪");
  gripper_name->setMinimumWidth(52);
  gripper_slider_ = new QSlider(Qt::Horizontal);
  gripper_slider_->setRange(0, 1000);
  gripper_slider_->setValue(1000);
  gripper_value_label_ = new QLabel("张开 1000");
  gripper_value_label_->setMinimumWidth(72);
  gripper_row->addWidget(gripper_name);
  gripper_row->addWidget(gripper_slider_);
  gripper_row->addWidget(gripper_value_label_);
  layout->addLayout(gripper_row);
  connect(gripper_slider_, &QSlider::sliderPressed, this, &JointSliderWindow::onSliderPressed);
  connect(gripper_slider_, &QSlider::sliderReleased, this, &JointSliderWindow::onSliderReleased);
  connect(gripper_slider_, &QSlider::valueChanged, this, &JointSliderWindow::onSliderChanged);

  // 升降柱 (lifter_1)：拖动滑条实时下发高度，服务不可用时自动置灰
  QHBoxLayout* lift_row = new QHBoxLayout();
  QLabel* lift_name = new QLabel("升降柱");
  lift_name->setMinimumWidth(52);
  lift_slider_ = new QSlider(Qt::Horizontal);
  lift_slider_->setRange(0, 600);
  lift_slider_->setValue(0);
  lift_slider_->setEnabled(false);
  lift_value_label_ = new QLabel("0 mm");
  lift_value_label_->setMinimumWidth(72);
  lift_status_label_ = new QLabel("未连接");
  lift_status_label_->setMinimumWidth(64);
  lift_stop_button_ = new QPushButton("急停");
  lift_stop_button_->setEnabled(false);
  lift_row->addWidget(lift_name);
  lift_row->addWidget(lift_slider_);
  lift_row->addWidget(lift_value_label_);
  lift_row->addWidget(lift_status_label_);
  lift_row->addWidget(lift_stop_button_);
  layout->addLayout(lift_row);
  connect(lift_slider_, &QSlider::sliderPressed, this, &JointSliderWindow::onSliderPressed);
  connect(lift_slider_, &QSlider::sliderReleased, this, &JointSliderWindow::onSliderReleased);
  connect(lift_slider_, &QSlider::valueChanged, this, &JointSliderWindow::onSliderChanged);
  connect(lift_stop_button_, &QPushButton::clicked, this, &JointSliderWindow::liftEmergencyStop);

  // 安全平面红点 (deck_top 系)：方向键步进 + 执行 / 急停。
  // agx_plane_target 节点没起时整行自动置灰。
  QHBoxLayout* plane_row = new QHBoxLayout();
  QLabel* plane_name = new QLabel("平面红点");
  plane_name->setMinimumWidth(52);
  plane_value_label_ = new QLabel("x ---  y --- mm");
  plane_value_label_->setMinimumWidth(150);
  plane_status_label_ = new QLabel("未连接");
  plane_status_label_->setMinimumWidth(72);
  plane_row->addWidget(plane_name);
  plane_row->addWidget(plane_value_label_);
  plane_row->addWidget(plane_status_label_);
  layout->addLayout(plane_row);

  QHBoxLayout* plane_dir_row = new QHBoxLayout();
  const char* dir_text[4] = {"←", "→", "↑", "↓"};
  const char* dir_tip[4] = {"deck_top -X（朝车尾）", "deck_top +X（朝车头）",
                            "deck_top +Y（左侧）", "deck_top -Y（右侧）"};
  for (int i = 0; i < 4; ++i)
  {
    plane_dir_buttons_[i] = new QPushButton(QString::fromUtf8(dir_text[i]));
    plane_dir_buttons_[i]->setFixedWidth(40);
    plane_dir_buttons_[i]->setToolTip(QString::fromUtf8(dir_tip[i]));
    plane_dir_buttons_[i]->setEnabled(false);
    plane_dir_row->addWidget(plane_dir_buttons_[i]);
    connect(plane_dir_buttons_[i], &QPushButton::clicked, this, &JointSliderWindow::planeNudge);
  }
  QLabel* step_name = new QLabel("步长");
  plane_step_spin_ = new QSpinBox();
  plane_step_spin_->setRange(1, 200);
  plane_step_spin_->setValue(20);
  plane_step_spin_->setSuffix(" mm");
  plane_step_spin_->setToolTip("每次按键红点移动的距离");
  plane_dir_row->addWidget(step_name);
  plane_dir_row->addWidget(plane_step_spin_);
  plane_reset_button_ = new QPushButton("回中心");
  plane_reset_button_->setEnabled(false);
  plane_dir_row->addWidget(plane_reset_button_);
  layout->addLayout(plane_dir_row);
  connect(plane_reset_button_, &QPushButton::clicked, this, &JointSliderWindow::planeReset);

  QHBoxLayout* plane_act_row = new QHBoxLayout();
  plane_go_button_ = new QPushButton("执行 GO（夹爪到红点）");
  plane_go_button_->setEnabled(false);
  plane_stop_button_ = new QPushButton("急停");
  plane_stop_button_->setEnabled(false);
  plane_act_row->addWidget(plane_go_button_);
  plane_act_row->addWidget(plane_stop_button_);
  layout->addLayout(plane_act_row);
  connect(plane_go_button_, &QPushButton::clicked, this, &JointSliderWindow::planeGo);
  connect(plane_stop_button_, &QPushButton::clicked, this, &JointSliderWindow::planeStop);

  // 夹取：一次完成「MoveIt 移到红点 + 闭合夹爪」。任务在 agx_plane_target 的
  // /plane_target/grasp 里做（它持有机械臂规划的安全区约束），这里只发一条
  // 松耦合服务调用。闭合位置写在参数里、每次夹取时重读，所以换物品只改这个
  // 数字，不用重启任何节点。agx_plane_target 没起 / 版本旧时整行自动置灰。
  QHBoxLayout* grasp_row = new QHBoxLayout();
  QLabel* grasp_name = new QLabel("夹取");
  grasp_name->setMinimumWidth(52);
  grasp_position_spin_ = new QSpinBox();
  grasp_position_spin_->setRange(0, 1000);
  grasp_position_spin_->setValue(0);
  grasp_position_spin_->setPrefix("闭合位置 ");
  grasp_position_spin_->setToolTip(
      "夹爪闭合目标 0~1000（1000 = 张开，0 = 闭合）\n"
      "0 = 尽量闭合，靠力度顶住物体，各种尺寸都能夹（推荐）\n"
      "夹不动 / 易碎 / 已夹着的物品：适当调大，让手指提前停\n"
      "每次点「夹取」时重读，不用重启");
  grasp_button_ = new QPushButton("夹取（移到红点后闭合）");
  grasp_button_->setToolTip(
      "先张开夹爪 → MoveIt 规划移到红点 → 闭合到「闭合位置」\n"
      "然后等夹爪真的停下来并回报实际读数。执行期间点「急停」可中断。");
  grasp_button_->setEnabled(false);
  grasp_status_label_ = new QLabel("未连接");
  grasp_status_label_->setMinimumWidth(72);
  grasp_status_label_->setToolTip(
      "夹取进度：夹取中 / 已完成 / 失败 / 已拒绝。详细读数看窗口最上面那行。");
  grasp_row->addWidget(grasp_name);
  grasp_row->addWidget(grasp_position_spin_);
  grasp_row->addWidget(grasp_button_);
  grasp_row->addWidget(grasp_status_label_);
  layout->addLayout(grasp_row);
  connect(grasp_button_, &QPushButton::clicked, this, &JointSliderWindow::planeGrasp);

  for (int i = 0; i < 6; ++i)
  {
    QHBoxLayout* row = new QHBoxLayout();
    QLabel* name = new QLabel(QString::fromStdString(kJointNames[i]));
    name->setMinimumWidth(52);
    sliders_[i] = new QSlider(Qt::Horizontal);
    sliders_[i]->setRange(radToSlider(lower_[i]), radToSlider(upper_[i]));
    sliders_[i]->setValue(0);
    value_labels_[i] = new QLabel("0.0 deg");
    value_labels_[i]->setMinimumWidth(72);
    row->addWidget(name);
    row->addWidget(sliders_[i]);
    row->addWidget(value_labels_[i]);
    layout->addLayout(row);

    connect(sliders_[i], &QSlider::sliderPressed, this, &JointSliderWindow::onSliderPressed);
    connect(sliders_[i], &QSlider::sliderReleased, this, &JointSliderWindow::onSliderReleased);
    connect(sliders_[i], &QSlider::valueChanged, this, &JointSliderWindow::onSliderChanged);
  }

  connect(timer_, &QTimer::timeout, this, &JointSliderWindow::publishPending);
  timer_->start(50);
  setWindowTitle("控制台");
  resize(640, 620);
  setupRos();
}

void JointSliderWindow::setupRos()
{
  urdf::Model model;
  if (model.initParam("robot_description"))
  {
    for (int i = 0; i < 6; ++i)
    {
      urdf::JointConstSharedPtr joint = model.getJoint(kJointNames[i]);
      if (!joint || !joint->limits)
        continue;
      lower_[i] = joint->limits->lower;
      upper_[i] = joint->limits->upper;
      updating_ = true;
      sliders_[i]->setRange(radToSlider(lower_[i]), radToSlider(upper_[i]));
      updating_ = false;
    }
  }

  command_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
      "/xarm/xarm6_traj_controller/command", 1);
  gripper_pub_ = nh_.advertise<dh_gripper_msgs::GripperCtrl>("/gripper/ctrl", 1);
  gripper_joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/gripper_slider/joint_states", 1);
  joint_sub_ = nh_.subscribe("/xarm/joint_states", 1, &JointSliderWindow::jointStateCb, this);
  gripper_sub_ = nh_.subscribe("/gripper/states", 1, &JointSliderWindow::gripperStateCb, this);
  arm_state_sub_ = nh_.subscribe("/xarm/xarm_states", 1, &JointSliderWindow::armStateCb, this);
  teach_start_client_ = nh_.serviceClient<std_srvs::Trigger>("/admittance_teach/start");
  teach_stop_client_ = nh_.serviceClient<std_srvs::Trigger>("/admittance_teach/stop");
  clear_err_client_ = nh_.serviceClient<xarm_msgs::ClearErr>("/xarm/clear_err");
  set_mode_client_ = nh_.serviceClient<xarm_msgs::SetInt16>("/xarm/set_mode");
  set_state_client_ = nh_.serviceClient<xarm_msgs::SetInt16>("/xarm/set_state");
  lift_sub_ = nh_.subscribe("/lifter_1/LiftMotorStatePub", 1, &JointSliderWindow::liftStateCb, this);
  lift_client_ = nh_.serviceClient<bt_task_msgs::LiftMotorSrv>("/lifter_1/LiftingMotorService");

  // 安全平面红点：话题/服务都是松耦合字符串接口，agx_plane_target 没启动时
  // ensurePlaneReady() 会把这一行置灰，不影响机械臂其它功能。
  plane_nudge_pub_ = nh_.advertise<geometry_msgs::Vector3>("/plane_target/nudge", 1);
  plane_target_sub_ = nh_.subscribe("/plane_target/target", 1, &JointSliderWindow::planeTargetCb, this);
  plane_go_client_ = nh_.serviceClient<std_srvs::Trigger>("/plane_target/go");
  plane_stop_client_ = nh_.serviceClient<std_srvs::Trigger>("/plane_target/stop");
  plane_reset_client_ = nh_.serviceClient<std_srvs::Trigger>("/plane_target/reset");
  plane_home_client_ = nh_.serviceClient<std_srvs::Trigger>("/plane_target/home");
  plane_grasp_client_ = nh_.serviceClient<std_srvs::Trigger>("/plane_target/grasp");
  // 夹取是异步的（服务立刻返回，运动在节点自己的线程里跑），所以进度只能靠
  // 这个话题回传。格式 "<state>|<text>"，state ∈ busy/ok/fail。
  grasp_status_sub_ = nh_.subscribe("/plane_target/grasp_status", 5,
                                    &JointSliderWindow::graspStatusCb, this);
}

void JointSliderWindow::jointStateCb(const sensor_msgs::JointState::ConstPtr& msg)
{
  std::array<double, 6> next{};
  for (int i = 0; i < 6; ++i)
  {
    auto it = std::find(msg->name.begin(), msg->name.end(), kJointNames[i]);
    if (it == msg->name.end())
      return;
    next[i] = msg->position[static_cast<size_t>(std::distance(msg->name.begin(), it))];
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_ = next;
    have_state_ = true;
  }
  if (teaching_.load())
    QMetaObject::invokeMethod(this, "applyTeachFollow", Qt::QueuedConnection);
  else if (!synced_)
    QMetaObject::invokeMethod(this, "applyInitialState", Qt::QueuedConnection);
}

void JointSliderWindow::armStateCb(const xarm_msgs::RobotMsg::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  arm_state_ = msg->state;
  arm_mode_ = msg->mode;
  arm_err_ = msg->err;
  have_arm_state_ = true;
}

bool JointSliderWindow::ensureArmReady()
{
  int state = 4;
  int mode = 0;
  int err = 0;
  bool have = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    have = have_arm_state_;
    state = arm_state_;
    mode = arm_mode_;
    err = arm_err_;
  }
  if (err == 1 || err == 2 || err == 3)
  {
    const char* reason = err == 1 ? "急停按钮被按下" : err == 2 ? "控制柜急停IO被触发" : "三态开关急停被按下";
    status_label_->setText(QString("机械臂被急停锁住：%1（err=%2）。松开后才能运动").arg(reason).arg(err));
    return false;
  }
  if (mode == 2 || mode == 3)
  {
    status_label_->setText("当前是拖动示教，请先点结束再使用滑条");
    return false;
  }
  // mode 0 is position mode. The controller switches it to servo. Calling
  // set_mode from here stops the arm on every slider tick.
  if (!have || (err == 0 && (state == 0 || state == 1 || state == 2)))
    return true;
  if ((ros::Time::now() - last_recover_).toSec() < 2.0)
  {
    status_label_->setText(QString("机械臂未就绪 state=%1 mode=%2 err=%3").arg(state).arg(mode).arg(err));
    return false;
  }
  last_recover_ = ros::Time::now();

  xarm_msgs::ClearErr clear;
  if (!clear_err_client_.call(clear) || clear.response.ret < 0)
  {
    status_label_->setText(QString("清除报错 %1 失败").arg(err));
    return false;
  }
  xarm_msgs::SetInt16 mode_srv;
  mode_srv.request.data = 1;
  if (!set_mode_client_.call(mode_srv) || mode_srv.response.ret < 0)
  {
    status_label_->setText("恢复伺服模式失败");
    return false;
  }
  xarm_msgs::SetInt16 state_srv;
  state_srv.request.data = 0;
  if (!set_state_client_.call(state_srv) || state_srv.response.ret < 0)
  {
    status_label_->setText("启动机械臂失败");
    return false;
  }
  ros::Duration(0.3).sleep();
  status_label_->setText(err == 0
      ? "机械臂已重新启动"
      : QString("已清除报错 %1（22 自碰撞，31 碰撞电流）并重新启动").arg(err));
  return true;
}

void JointSliderWindow::gripperStateCb(const dh_gripper_msgs::GripperState::ConstPtr& msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gripper_position_ = msg->position;
  }
  if (teaching_.load())
    QMetaObject::invokeMethod(this, "applyGripperTeach", Qt::QueuedConnection);
  else if (!gripper_synced_)
    QMetaObject::invokeMethod(this, "applyGripperState", Qt::QueuedConnection);
}

void JointSliderWindow::applyGripperState()
{
  if (gripper_synced_ || driving_)
    return;
  float position;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    position = gripper_position_;
  }
  int value = std::max(0, std::min(1000, static_cast<int>(std::lround(position))));
  updating_ = true;
  gripper_slider_->setValue(value);
  gripper_value_label_->setText(value >= 900 ? QString("张开 %1").arg(value) : QString("闭合 %1").arg(value));
  updating_ = false;
  gripper_synced_ = true;
}

void JointSliderWindow::applyGripperTeach()
{
  if (!teaching_.load() || driving_)
    return;
  float position;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    position = gripper_position_;
  }
  int value = std::max(0, std::min(1000, static_cast<int>(std::lround(position))));
  updating_ = true;
  gripper_slider_->setValue(value);
  gripper_value_label_->setText(value >= 900 ? QString("张开 %1").arg(value) : QString("闭合 %1").arg(value));
  updating_ = false;
}

void JointSliderWindow::liftStateCb(const bt_task_msgs::LiftMotorMsg::ConstPtr& msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    lift_height_ = static_cast<int>(msg->backHeight);
    lift_reached_ = msg->reachTargetPos;
    lift_initialized_ = msg->initState;
    lift_fault_ = msg->overflowI || msg->overflowV || msg->overLoad || msg->underVol ||
                  msg->encodeErr || msg->posBiasOver || msg->fpsError > 1;
    if (msg->upLimitHeight > msg->downLimitHeight)
    {
      lift_min_ = static_cast<int>(msg->downLimitHeight);
      lift_max_ = static_cast<int>(msg->upLimitHeight);
    }
  }
  QMetaObject::invokeMethod(this, "applyLiftState", Qt::QueuedConnection);
}

void JointSliderWindow::applyLiftState()
{
  int height, min_h, max_h;
  bool reached, initialized, fault;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    height = lift_height_;
    min_h = lift_min_;
    max_h = lift_max_;
    reached = lift_reached_;
    initialized = lift_initialized_;
    fault = lift_fault_;
  }

  if (max_h > min_h && (lift_slider_->minimum() != min_h || lift_slider_->maximum() != max_h))
  {
    updating_ = true;
    lift_slider_->setRange(min_h, max_h);
    updating_ = false;
  }

  if (!lift_dragging_)
  {
    const int v = std::max(lift_slider_->minimum(), std::min(lift_slider_->maximum(), height));
    updating_ = true;
    lift_slider_->setValue(v);
    lift_value_label_->setText(QString("%1 mm").arg(v));
    updating_ = false;
  }

  QString state_text;
  if (fault)
    state_text = "故障";
  else if (!initialized)
    // initState=false 现在有两种来源：真在跑初始化，或者 830ABS 断电后位置基准丢失、
    // 节点拒绝位置指令（见 lifting_ctrl_service_node.py 的 CheckFrameAtStartup）。
    // 两种情况下高度读数都不可信，所以滑条必须置灰。
    state_text = "未标定";
  else if (reached)
    state_text = "已到达";
  else
    state_text = "运行中";
  lift_status_label_->setText(state_text);
  // 坐标系不可信时禁用滑条：拖动只会发一条注定被拒的位置指令。
  // 急停按钮保持可用（-2 不受门禁限制）。
  lift_slider_->setEnabled(lift_connected_ && initialized && !fault);
  if (!initialized && !fault)
  {
    lift_value_label_->setText(QString("需 lift init"));
    status_label_->setText("升降柱坐标系未标定（断电后 830ABS 位置基准丢失）："
                           "滑条已锁定，请执行 'lift init' 下降到下限位重新建立坐标系");
  }
  lift_synced_ = true;
}

bool JointSliderWindow::ensureLiftReady()
{
  if (lift_client_.exists())
  {
    if (!lift_connected_)
    {
      lift_connected_ = true;
      lift_slider_->setEnabled(true);
      lift_stop_button_->setEnabled(true);
    }
    return true;
  }
  if (lift_connected_ || lift_status_label_->text() != "未连接")
  {
    lift_connected_ = false;
    lift_slider_->setEnabled(false);
    lift_stop_button_->setEnabled(false);
    lift_status_label_->setText("未连接");
  }
  return false;
}

void JointSliderWindow::sendLift(bool force)
{
  if (!ensureLiftReady())
  {
    status_label_->setText("升降柱未连接：请确认 lifting_ctrl 已启动（setup_arm.launch 默认会启动）");
    return;
  }

  const int target = lift_slider_->value();
  const ros::Time now = ros::Time::now();
  if (!force)
  {
    if (target == lift_last_sent_)
      return;
    // 拖动中实时下发，但限流到 10Hz，避免刷爆串口
    if (!lift_last_send_.isZero() && (now - lift_last_send_).toSec() < 0.1)
      return;
  }

  bt_task_msgs::LiftMotorSrv srv;
  srv.request.val = target;
  srv.request.mode = 0;
  if (!lift_client_.call(srv))
  {
    lift_status_label_->setText("调用失败");
    lift_connected_ = false;
    return;
  }
  lift_last_send_ = now;
  lift_last_sent_ = target;
  status_label_->setText(QString("升降柱目标 %1 mm（state=%2）").arg(target).arg(srv.response.state));
}

void JointSliderWindow::liftEmergencyStop()
{
  if (!ensureLiftReady())
  {
    status_label_->setText("升降柱未连接，无法急停");
    return;
  }
  bt_task_msgs::LiftMotorSrv srv;
  srv.request.val = 0;
  srv.request.mode = -2;
  if (!lift_client_.call(srv))
  {
    status_label_->setText("升降柱急停调用失败");
    return;
  }
  lift_last_sent_ = -1;  // 允许之后重新发送目标
  status_label_->setText(QString("已发送升降柱急停（state=%1）").arg(srv.response.state));
}

// --- 安全平面红点 -------------------------------------------------------
void JointSliderWindow::planeTargetCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    plane_x_ = msg->pose.position.x;
    plane_y_ = msg->pose.position.y;
    plane_have_ = true;
  }
  QMetaObject::invokeMethod(this, "applyPlaneTarget", Qt::QueuedConnection);
}

void JointSliderWindow::applyPlaneTarget()
{
  double x, y;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!plane_have_)
      return;
    x = plane_x_;
    y = plane_y_;
  }
  plane_value_label_->setText(QString("x %1  y %2 mm")
                                  .arg(x * 1000.0, 0, 'f', 0)
                                  .arg(y * 1000.0, 0, 'f', 0));
}

bool JointSliderWindow::ensurePlaneReady()
{
  if (plane_go_client_.exists())
  {
    if (!plane_connected_)
    {
      plane_connected_ = true;
      for (int i = 0; i < 4; ++i)
        plane_dir_buttons_[i]->setEnabled(true);
      plane_reset_button_->setEnabled(true);
      plane_go_button_->setEnabled(true);
      plane_stop_button_->setEnabled(true);
      plane_status_label_->setText("已连接");
    }
    return true;
  }
  if (plane_connected_ || plane_status_label_->text() != "未连接")
  {
    plane_connected_ = false;
    for (int i = 0; i < 4; ++i)
      plane_dir_buttons_[i]->setEnabled(false);
    plane_reset_button_->setEnabled(false);
    plane_go_button_->setEnabled(false);
    plane_stop_button_->setEnabled(false);
    plane_status_label_->setText("未连接");
  }
  return false;
}

void JointSliderWindow::planeNudge()
{
  QPushButton* button = qobject_cast<QPushButton*>(sender());
  if (!button)
    return;
  if (!ensurePlaneReady())
  {
    status_label_->setText("平面红点未连接：请确认 plane_target_marker 已启动"
                           "（setup_arm.launch 用 open_plane_target:=false 关闭过）");
    return;
  }
  const double step = plane_step_spin_->value() / 1000.0;
  geometry_msgs::Vector3 nudge;
  if (button == plane_dir_buttons_[0])
    nudge.x = -step;  // ← -X
  else if (button == plane_dir_buttons_[1])
    nudge.x = step;   // → +X
  else if (button == plane_dir_buttons_[2])
    nudge.y = step;   // ↑ +Y
  else
    nudge.y = -step;  // ↓ -Y
  plane_nudge_pub_.publish(nudge);
  applyPlaneTarget();  // 节点回声会校正越界钳位，这里先给出即时反馈
  status_label_->setText(QString("红点移动 %1 mm（x %2, y %3）")
                             .arg(step * 1000.0, 0, 'f', 0)
                             .arg(nudge.x * 1000.0, 0, 'f', 0)
                             .arg(nudge.y * 1000.0, 0, 'f', 0));
}

void JointSliderWindow::planeGo()
{
  if (!ensurePlaneReady())
  {
    status_label_->setText("平面红点未连接，无法执行");
    return;
  }
  if (!ensureArmReady())
    return;  // 急停锁住 / 拖动示教等状态已在 status_label_ 里说明
  std_srvs::Trigger srv;
  if (!plane_go_client_.call(srv))
  {
    status_label_->setText("调用 /plane_target/go 失败");
    return;
  }
  status_label_->setText(QString("红点：%1").arg(QString::fromStdString(srv.response.message)));
}

void JointSliderWindow::planeStop()
{
  if (!ensurePlaneReady())
  {
    status_label_->setText("平面红点未连接，无法急停");
    return;
  }
  std_srvs::Trigger srv;
  if (!plane_stop_client_.call(srv))
  {
    status_label_->setText("调用 /plane_target/stop 失败");
    return;
  }
  status_label_->setText(QString("红点：%1").arg(QString::fromStdString(srv.response.message)));
}

void JointSliderWindow::planeReset()
{
  if (!ensurePlaneReady())
  {
    status_label_->setText("平面红点未连接");
    return;
  }
  std_srvs::Trigger srv;
  if (!plane_reset_client_.call(srv))
  {
    status_label_->setText("调用 /plane_target/reset 失败");
    return;
  }
  status_label_->setText(QString::fromStdString(srv.response.message));
}

// --- 夹取：移到红点 + 闭合 -------------------------------------------------
void JointSliderWindow::graspStatusCb(const std_msgs::String::ConstPtr& msg)
{
  const size_t sep = msg->data.find('|');
  {
    std::lock_guard<std::mutex> lock(mutex_);
    grasp_state_ = (sep == std::string::npos) ? msg->data : msg->data.substr(0, sep);
    grasp_text_ = (sep == std::string::npos) ? std::string() : msg->data.substr(sep + 1);
  }
  QMetaObject::invokeMethod(this, "applyGraspStatus", Qt::QueuedConnection);
}

void JointSliderWindow::applyGraspStatus()
{
  std::string state, text;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = grasp_state_;
    text = grasp_text_;
  }
  // 夹取跑在 agx_plane_target 的线程里，这里只做显示；busy 期间禁掉按钮，
  // 避免连点把同一个 xarm6 控制器塞进两条指令。
  grasp_busy_ = (state == "busy");
  QString label;
  QString colour;
  if (state == "busy")
  {
    label = "夹取中";
    colour = "#c06000";
  }
  else if (state == "ok")
  {
    label = "已完成";
    colour = "#107010";
  }
  else if (state == "fail")
  {
    label = "失败";
    colour = "#c00000";
  }
  else
  {
    label = "已连接";
    colour = "";
  }
  grasp_status_label_->setText(label);
  grasp_status_label_->setStyleSheet(colour.isEmpty() ? QString() : QString("color: %1").arg(colour));
  if (!text.empty())
    status_label_->setText(QString::fromStdString(text));
  if (!grasp_busy_)
  {
    // 夹取直接改了夹爪位置，而夹爪滑条只在启动时同步一次。让它重新同步，
    // 否则松手后再拖滑条会从 1000 跳到实际值、看着像失控。
    gripper_synced_ = false;
    grasp_button_->setEnabled(grasp_connected_);
  }
}

bool JointSliderWindow::ensureGraspReady()
{
  if (plane_grasp_client_.exists())
  {
    if (!grasp_connected_)
    {
      grasp_connected_ = true;
      grasp_button_->setEnabled(!grasp_busy_);
      if (grasp_status_label_->text() == "未连接")
        grasp_status_label_->setText("已连接");
    }
    return true;
  }
  if (grasp_connected_ || grasp_status_label_->text() != "未连接")
  {
    grasp_connected_ = false;
    grasp_busy_ = false;
    grasp_button_->setEnabled(false);
    grasp_status_label_->setText("未连接");
  }
  return false;
}

void JointSliderWindow::planeGrasp()
{
  if (!ensureGraspReady())
  {
    status_label_->setText("夹取未连接：请确认 plane_target_marker 已启动"
                           "（setup_arm.launch 用 open_plane_target:=false 关闭过）");
    return;
  }
  if (grasp_busy_)
  {
    status_label_->setText("上一次夹取还在进行，请等它结束（要中断点「急停」）");
    return;
  }
  if (!ensureArmReady())
    return;  // 急停锁住 / 拖动示教等状态已在 status_label_ 里说明

  // 闭合参数走参数服务器而不是自定义 srv：和 ~rpy_deg 同一个套路，任务层
  // 每次夹取重读，改数字不用重启任何节点，也不用给 start 层加新消息依赖。
  nh_.setParam("/plane_target_marker/grasp_position", grasp_position_spin_->value());
  std_srvs::Trigger srv;
  if (!plane_grasp_client_.call(srv))
  {
    status_label_->setText("调用 /plane_target/grasp 失败");
    return;
  }
  status_label_->setText(QString("夹取：%1").arg(QString::fromStdString(srv.response.message)));
  if (!srv.response.success)
  {
    // 节点拒收（例如上一次夹取还在跑）。别把自己也标成 busy，否则要等
    // 状态话题推新消息才恢复，而被拒时根本不一定会推。
    grasp_status_label_->setText("已拒绝");
    grasp_status_label_->setStyleSheet("color: #c00000");
    return;
  }
  // 立即置 busy 并禁用按钮，不等话题回声：服务已经返回，流程确实开始了。
  // 禁按钮是为了防止连点把同一个 xarm6 控制器塞进两条指令。
  grasp_busy_ = true;
  grasp_button_->setEnabled(false);
  grasp_status_label_->setText("夹取中");
  grasp_status_label_->setStyleSheet("color: #c06000");
}

void JointSliderWindow::applyInitialState()
{
  if (synced_ || driving_)
    return;
  std::array<double, 6> current;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_state_)
      return;
    current = current_;
  }
  updating_ = true;
  for (int i = 0; i < 6; ++i)
  {
    int value = std::max(sliders_[i]->minimum(), std::min(sliders_[i]->maximum(), radToSlider(current[i])));
    sliders_[i]->setValue(value);
    value_labels_[i]->setText(QString::number(value / 10.0, 'f', 1) + " deg");
  }
  updating_ = false;
  synced_ = true;
  status_label_->setText("拖动滑条会实时驱动真实机械臂");
}

void JointSliderWindow::applyTeachFollow()
{
  if (!teaching_.load() || driving_)
    return;
  std::array<double, 6> current;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_state_)
      return;
    current = current_;
  }
  updating_ = true;
  for (int i = 0; i < 6; ++i)
  {
    int value = std::max(sliders_[i]->minimum(), std::min(sliders_[i]->maximum(), radToSlider(current[i])));
    sliders_[i]->setValue(value);
    value_labels_[i]->setText(QString::number(value / 10.0, 'f', 1) + " deg");
  }
  updating_ = false;
}

void JointSliderWindow::startTeach()
{
  if (!teach_start_client_.waitForExistence(ros::Duration(1.0)))
  {
    status_label_->setText("没有 /admittance_teach/start，请先启动 setup_arm.launch");
    return;
  }
  std_srvs::Trigger srv;
  if (!teach_start_client_.call(srv) || !srv.response.success)
  {
    status_label_->setText(QString::fromStdString(
        srv.response.message.empty() ? "进入拖动示教失败" : srv.response.message));
    return;
  }
  teaching_.store(true);
  pending_ = false;
  status_label_->setText(QString::fromStdString(srv.response.message));
}

void JointSliderWindow::stopTeach()
{
  if (!teach_stop_client_.waitForExistence(ros::Duration(1.0)))
  {
    status_label_->setText("没有 /admittance_teach/stop，请先启动 setup_arm.launch");
    return;
  }
  std_srvs::Trigger srv;
  if (!teach_stop_client_.call(srv) || !srv.response.success)
  {
    status_label_->setText(QString::fromStdString(
        srv.response.message.empty() ? "结束拖动示教失败" : srv.response.message));
    return;
  }
  applyTeachFollow();
  teaching_.store(false);
  pending_ = false;
  synced_ = true;
  status_label_->setText(QString::fromStdString(srv.response.message));
}

void JointSliderWindow::goToInitialPose()
{
  if (teaching_.load())
  {
    status_label_->setText("请先结束拖动示教，再回到初始位置");
    return;
  }
  pending_ = false;
  pending_gripper_ = false;
  updating_ = true;
  for (int i = 0; i < 6; ++i)
  {
    int value = std::max(sliders_[i]->minimum(), std::min(sliders_[i]->maximum(), radToSlider(kInitialArm[i])));
    sliders_[i]->setValue(value);
    value_labels_[i]->setText(QString::number(value / 10.0, 'f', 1) + " deg");
  }
  gripper_slider_->setValue(kInitialGripper);
  gripper_value_label_->setText("张开 1000");
  updating_ = false;
  synced_ = true;
  gripper_synced_ = true;
  sendGripper();
  // 优先走 MoveIt：/plane_target/home 会带着"末端全程留在黄色安全区内"的
  // 路径约束规划并执行。旧实现是一条裸关节轨迹直发控制器（无规划、无碰撞检查），
  // 复位时手臂会大幅摆动，正是要避免的危险动作。
  if (plane_home_client_.exists())
  {
    std_srvs::Trigger home_srv;
    if (plane_home_client_.call(home_srv))
    {
      status_label_->setText(QString("回到初始位置：%1")
                             .arg(QString::fromStdString(home_srv.response.message)));
      return;
    }
  }
  if (sendTarget())
    status_label_->setText("正在回到初始位置（直接下发，未经规划，路径不受黄色安全区约束）");
}

void JointSliderWindow::onSliderPressed()
{
  driving_ = true;
  if (sender() == lift_slider_)
    lift_dragging_ = true;
}

void JointSliderWindow::onSliderReleased()
{
  driving_ = false;
  if (sender() == lift_slider_)
  {
    lift_dragging_ = false;
    pending_lift_ = false;
    sendLift(true);  // 松手时强制下发一次，确保最终值生效
    return;
  }
  if (sender() == gripper_slider_)
  {
    pending_gripper_ = false;
    sendGripper();
    return;
  }
  pending_ = true;
  sendTarget();
}

void JointSliderWindow::onSliderChanged()
{
  if (updating_)
    return;
  QSlider* slider = qobject_cast<QSlider*>(sender());
  if (slider == lift_slider_)
  {
    lift_value_label_->setText(QString("%1 mm").arg(lift_slider_->value()));
    pending_lift_ = true;
    return;
  }
  if (slider == gripper_slider_)
  {
    int value = gripper_slider_->value();
    gripper_value_label_->setText(value >= 900 ? QString("张开 %1").arg(value) : QString("闭合 %1").arg(value));
    return;
  }
  for (int i = 0; i < 6; ++i)
  {
    if (sliders_[i] == slider)
    {
      value_labels_[i]->setText(QString::number(slider->value() / 10.0, 'f', 1) + " deg");
      break;
    }
  }
  if (!synced_ || teaching_.load())
    return;
  pending_ = true;
}

void JointSliderWindow::publishPending()
{
  ensureLiftReady();
  ensurePlaneReady();
  ensureGraspReady();
  if (pending_lift_)
  {
    pending_lift_ = false;
    sendLift();
  }
  // The gripper is sent on slider release only. Publishing every 50 ms
  // filled the driver queue with intermediate openings.
  if (!pending_)
    return;
  pending_ = false;
  sendTarget();
}

void JointSliderWindow::sendGripper()
{
  dh_gripper_msgs::GripperCtrl cmd;
  cmd.initialize = false;
  cmd.position = gripper_slider_->value();
  cmd.force = teaching_.load() ? 20.0f : 50.0f;
  cmd.speed = std::max(10, speed_slider_->value());
  gripper_pub_.publish(cmd);

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.push_back("finger_joint");
  joint_state.position.push_back((1000.0 - cmd.position) / 1000.0 * 0.6524);
  gripper_joint_pub_.publish(joint_state);
  if (!teaching_.load())
    status_label_->setText(QString("夹爪目标 %1").arg(cmd.position, 0, 'f', 0));
}

double JointSliderWindow::sliderRadians(int index) const
{
  double rad = sliders_[index]->value() / 10.0 * M_PI / 180.0;
  return std::max(lower_[index], std::min(upper_[index], rad));
}

bool JointSliderWindow::sendTarget()
{
  if (teaching_.load())
  {
    status_label_->setText("拖动示教中，滑条不会下发位置");
    return false;
  }
  if (!synced_)
  {
    status_label_->setText("正在同步当前关节角，请稍后再拖滑条");
    return false;
  }
  if (!ensureArmReady())
    return false;
  std::array<double, 6> current;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_state_)
    {
      status_label_->setText("尚未收到 /xarm/joint_states");
      return false;
    }
    current = current_;
  }

  trajectory_msgs::JointTrajectory traj;
  traj.joint_names.assign(kJointNames.begin(), kJointNames.end());
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.resize(6);
  double max_delta = 0.0;
  for (int i = 0; i < 6; ++i)
  {
    point.positions[i] = sliderRadians(i);
    max_delta = std::max(max_delta, std::abs(point.positions[i] - current[i]));
  }
  if (max_delta < 0.002)
    return true;

  double speed = std::max(0.1, speed_slider_->value() / 100.0);
  double duration = std::max(max_delta / (0.8 * speed), 0.08);
  point.time_from_start = ros::Duration(duration);
  traj.points.push_back(point);
  command_pub_.publish(traj);
  status_label_->setText(QString("正在跟踪滑条，约 %1 s").arg(duration, 0, 'f', 2));
  return true;
}

}  // namespace agx_xarm_bringup

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_slider_gui");
  QApplication app(argc, argv);
  agx_xarm_bringup::JointSliderWindow window;
  window.show();
  ros::AsyncSpinner spinner(2);
  spinner.start();
  const int code = app.exec();
  ros::shutdown();
  return code;
}
