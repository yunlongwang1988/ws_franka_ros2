#pragma once

#include <Eigen/Dense>
#include <atomic>
#include <memory>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <franka_example_controllers/robot_utils.hpp>
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

struct AxisState {
  double pos;
  double vel;
  double acc;
};

class CartesianPoseExampleController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;

  const bool k_elbow_activated_{false};
  bool init_flag_{true};

  std::string robot_description_;
  std::string arm_id_;

  // 位置：每轴独立临界阻尼二阶
  AxisState axis_[3];

  // 姿态：临界阻尼二阶角速度向量 + 指数映射积分
  Eigen::Quaterniond cmd_ori_;
  Eigen::Vector3d ang_vel_;
  Eigen::Vector3d ang_acc_;

  // RT 线程目标（由增量计算得出）
  double goal_pos_[3];
  Eigen::Quaterniond goal_ori_;

  // --- 增量式遥操作状态 ---
  // Sigma pose 订阅（lock-free 双缓冲）
  struct SigmaPose {
    Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond ori{Eigen::Quaterniond::Identity()};
  };
  SigmaPose sigma_buf_[2];
  std::atomic<int> sigma_active_buf_{0};
  std::atomic<bool> sigma_has_new_{false};

  // Clutch 状态
  std::atomic<bool> clutch_pressed_{false};
  bool clutch_was_pressed_{false};
  bool anchored_{false};

  // Anchor（首次收到 sigma 数据时锚定）
  Eigen::Vector3d sigma_anchor_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond sigma_anchor_ori_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d franka_anchor_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond franka_anchor_ori_{Eigen::Quaterniond::Identity()};

  // 最新 sigma 位姿（RT线程读取）
  Eigen::Vector3d sigma_latest_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond sigma_latest_ori_{Eigen::Quaterniond::Identity()};

  // 参数：各轴独立缩放（含方向翻转，负值表示取反）
  // sigma7 工作空间 ~±0.04m，franka 有效范围 ~0.6m
  double scale_x_{-5.0};   // sigma X 取反映射到 franka X
  double scale_y_{-2.5};   // sigma Y 取反映射到 franka Y
  double scale_z_{6.5};    // sigma Z 同向映射到 franka Z
  double scale_ori_{0.6};

  // 工作空间边界 [min, max]（franka base frame，单位 m）
  double ws_min_[3] = {0.2, -0.4, -0.22};
  double ws_max_[3] = {0.75,  0.4,  0.7};

  // 位置参数：二阶滤波 + 饱和钳位
  double wn_pos_;
  double zeta_pos_;
  double max_vel_;
  double max_acc_;
  double max_jerk_;

  // 姿态参数
  double wn_ori_;
  double zeta_ori_;
  double max_ang_vel_;
  double max_ang_acc_;
  double max_ang_jerk_;

  static constexpr double kDt = 0.001;  // 1kHz

  // 订阅
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sigma_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sigma_button_sub_;

  void sigmaPosCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void sigmaButtonCb(const sensor_msgs::msg::Joy::SharedPtr msg);
  void stepAxis(AxisState& state, double target);
  void stepOrientation();
};

}  // namespace franka_example_controllers
