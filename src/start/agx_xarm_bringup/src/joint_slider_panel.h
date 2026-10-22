#ifndef AGX_XARM_JOINT_SLIDER_PANEL_H
#define AGX_XARM_JOINT_SLIDER_PANEL_H

#include <ros/ros.h>
#include <bt_task_msgs/LiftMotorMsg.h>
#include <bt_task_msgs/LiftMotorSrv.h>
#include <sensor_msgs/JointState.h>
#include <dh_gripper_msgs/GripperCtrl.h>
#include <dh_gripper_msgs/GripperState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <xarm_msgs/ClearErr.h>
#include <xarm_msgs/RobotMsg.h>
#include <xarm_msgs/SetInt16.h>

#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QWidget>

#include <array>
#include <atomic>
#include <mutex>
#include <string>

class QPushButton;
class QTimer;

namespace agx_xarm_bringup
{
class JointSliderWindow : public QWidget
{
  Q_OBJECT
public:
  explicit JointSliderWindow(QWidget* parent = nullptr);

private Q_SLOTS:
  void onSliderPressed();
  void onSliderReleased();
  void onSliderChanged();
  void publishPending();
  void applyInitialState();
  void applyGripperState();
  void applyGripperTeach();
  void applyTeachFollow();
  void goToInitialPose();
  void startTeach();
  void stopTeach();
  void applyLiftState();
  void liftEmergencyStop();
  void planeNudge();
  void planeGo();
  void planeStop();
  void planeReset();
  void applyPlaneTarget();
  void planeGrasp();
  void applyGraspStatus();
  void graspStatusCb(const std_msgs::String::ConstPtr& msg);

private:
  void setupRos();
  void jointStateCb(const sensor_msgs::JointState::ConstPtr& msg);
  void gripperStateCb(const dh_gripper_msgs::GripperState::ConstPtr& msg);
  void liftStateCb(const bt_task_msgs::LiftMotorMsg::ConstPtr& msg);
  void sendGripper();
  bool sendTarget();
  bool ensureArmReady();
  bool ensureLiftReady();
  bool ensurePlaneReady();
  bool ensureGraspReady();
  void sendLift(bool force = false);
  double sliderRadians(int index) const;
  void armStateCb(const xarm_msgs::RobotMsg::ConstPtr& msg);
  void planeTargetCb(const geometry_msgs::PoseStamped::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber joint_sub_;
  ros::Subscriber gripper_sub_;
  ros::Subscriber arm_state_sub_;
  ros::Publisher command_pub_;
  ros::Publisher gripper_pub_;
  ros::Publisher gripper_joint_pub_;
  ros::Publisher plane_nudge_pub_;
  ros::ServiceClient teach_start_client_;
  ros::ServiceClient teach_stop_client_;
  ros::ServiceClient clear_err_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient set_state_client_;
  ros::ServiceClient lift_client_;
  ros::ServiceClient plane_go_client_;
  ros::ServiceClient plane_stop_client_;
  ros::ServiceClient plane_reset_client_;
  ros::ServiceClient plane_home_client_;
  ros::ServiceClient plane_grasp_client_;
  ros::Subscriber lift_sub_;
  ros::Subscriber plane_target_sub_;
  ros::Subscriber grasp_status_sub_;
  QTimer* timer_;

  std::array<QSlider*, 6> sliders_{};
  std::array<QLabel*, 6> value_labels_{};
  QSlider* gripper_slider_{nullptr};
  QLabel* gripper_value_label_;
  QSlider* speed_slider_;
  QLabel* status_label_;
  QSlider* lift_slider_{nullptr};
  QLabel* lift_value_label_{nullptr};
  QLabel* lift_status_label_{nullptr};
  QPushButton* lift_stop_button_{nullptr};

  // --- 安全平面红点 (agx_plane_target) ---
  QLabel* plane_value_label_{nullptr};
  QLabel* plane_status_label_{nullptr};
  QPushButton* plane_dir_buttons_[4]{};  // -x, +x, +y, -y
  QPushButton* plane_go_button_{nullptr};
  QPushButton* plane_stop_button_{nullptr};
  QPushButton* plane_reset_button_{nullptr};
  QSpinBox* plane_step_spin_{nullptr};
  bool plane_connected_;
  bool plane_have_;
  double plane_x_;
  double plane_y_;

  // --- 夹取 (plane_target/grasp: 移到红点后闭合) ---
  QSpinBox* grasp_position_spin_{nullptr};
  QPushButton* grasp_button_{nullptr};
  QLabel* grasp_status_label_{nullptr};
  bool grasp_connected_;
  bool grasp_busy_;
  std::string grasp_state_;
  std::string grasp_text_;

  std::array<double, 6> lower_{};
  std::array<double, 6> upper_{};
  std::array<double, 6> current_{};

  std::mutex mutex_;
  bool have_state_;
  bool synced_;
  bool gripper_synced_;
  bool driving_;
  bool pending_;
  bool pending_gripper_;
  bool updating_;
  std::atomic<bool> teaching_;
  bool have_arm_state_;
  int arm_state_;
  int arm_mode_;
  int arm_err_;
  ros::Time last_recover_;
  float gripper_position_;

  // --- 升降柱 (lifter_1) ---
  bool lift_connected_;
  bool lift_dragging_;
  bool lift_synced_;
  bool pending_lift_;
  int lift_height_;
  int lift_min_;
  int lift_max_;
  bool lift_reached_;
  bool lift_initialized_;
  bool lift_fault_;
  int lift_last_sent_;
  ros::Time lift_last_send_;

  static const std::array<std::string, 6> kJointNames;
};

}  // namespace agx_xarm_bringup

#endif
