#include <franka_example_controllers/cartesian_pose_example_controller.hpp>
#include <franka_example_controllers/default_robot_behavior_utils.hpp>

#include <cmath>
#include <string>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
CartesianPoseExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_command_interface_names();
  return config;
}

controller_interface::InterfaceConfiguration
CartesianPoseExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();
  config.names.push_back(arm_id_ + "/robot_time");
  return config;
}

void CartesianPoseExampleController::sigmaPosCb(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  int wb = 1 - sigma_active_buf_.load(std::memory_order_acquire);
  sigma_buf_[wb].pos = Eigen::Vector3d(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  sigma_buf_[wb].ori = Eigen::Quaterniond(
      msg->pose.orientation.w, msg->pose.orientation.x,
      msg->pose.orientation.y, msg->pose.orientation.z);
  if (sigma_buf_[wb].ori.norm() > 1e-6) {
    sigma_buf_[wb].ori.normalize();
  }
  sigma_active_buf_.store(wb, std::memory_order_release);
  sigma_has_new_.store(true, std::memory_order_release);
}

void CartesianPoseExampleController::sigmaButtonCb(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  if (msg->buttons.size() >= 2) {
    clutch_pressed_.store(msg->buttons[1] != 0, std::memory_order_release);
  }
}

void CartesianPoseExampleController::stepAxis(AxisState& state, double target) {
  double err = target - state.pos;
  double desired_acc = wn_pos_ * wn_pos_ * err - 2.0 * zeta_pos_ * wn_pos_ * state.vel;

  if (desired_acc > max_acc_) desired_acc = max_acc_;
  if (desired_acc < -max_acc_) desired_acc = -max_acc_;

  double acc_change = desired_acc - state.acc;
  double max_acc_change = max_jerk_ * kDt;
  if (acc_change > max_acc_change) acc_change = max_acc_change;
  if (acc_change < -max_acc_change) acc_change = -max_acc_change;
  state.acc += acc_change;

  state.vel += state.acc * kDt;
  if (state.vel > max_vel_) state.vel = max_vel_;
  if (state.vel < -max_vel_) state.vel = -max_vel_;

  state.pos += state.vel * kDt;
}

void CartesianPoseExampleController::stepOrientation() {
  Eigen::Quaterniond q_err = goal_ori_ * cmd_ori_.conjugate();
  if (q_err.w() < 0.0) {
    q_err.coeffs() *= -1.0;
  }
  q_err.normalize();
  Eigen::AngleAxisd aa(q_err);
  Eigen::Vector3d err_vec = aa.axis() * aa.angle();

  Eigen::Vector3d desired_acc =
      wn_ori_ * wn_ori_ * err_vec - 2.0 * zeta_ori_ * wn_ori_ * ang_vel_;
  double da = desired_acc.norm();
  if (da > max_ang_acc_) desired_acc *= (max_ang_acc_ / da);

  Eigen::Vector3d acc_change = desired_acc - ang_acc_;
  double ac = acc_change.norm();
  double max_ac = max_ang_jerk_ * kDt;
  if (ac > max_ac) acc_change *= (max_ac / ac);
  ang_acc_ += acc_change;

  ang_vel_ += ang_acc_ * kDt;
  double vn = ang_vel_.norm();
  if (vn > max_ang_vel_) ang_vel_ *= (max_ang_vel_ / vn);

  Eigen::Vector3d dtheta = ang_vel_ * kDt;
  double angle = dtheta.norm();
  Eigen::Quaterniond dq;
  if (angle > 1e-12) {
    dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, dtheta / angle));
  } else {
    dq = Eigen::Quaterniond::Identity();
  }
  cmd_ori_ = dq * cmd_ori_;
  cmd_ori_.normalize();
}

controller_interface::return_type CartesianPoseExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  if (init_flag_) {
    Eigen::Quaterniond ori;
    Eigen::Vector3d pos;
    std::tie(ori, pos) = franka_cartesian_pose_->getCurrentOrientationAndTranslation();

    for (int i = 0; i < 3; i++) {
      axis_[i].pos = pos(i);
      axis_[i].vel = 0.0;
      axis_[i].acc = 0.0;
      goal_pos_[i] = pos(i);
    }
    cmd_ori_ = ori;
    goal_ori_ = ori;
    ang_vel_ = Eigen::Vector3d::Zero();
    ang_acc_ = Eigen::Vector3d::Zero();
    anchored_ = false;

    RCLCPP_INFO(get_node()->get_logger(), "Initial pose: [%.3f, %.3f, %.3f]",
                pos.x(), pos.y(), pos.z());
    init_flag_ = false;
  }

  // 读取最新 sigma 位姿
  if (sigma_has_new_.load(std::memory_order_acquire)) {
    int rb = sigma_active_buf_.load(std::memory_order_acquire);
    sigma_latest_pos_ = sigma_buf_[rb].pos;
    sigma_latest_ori_ = sigma_buf_[rb].ori;
    sigma_has_new_.store(false, std::memory_order_release);

    if (!anchored_) {
      // 收到第一帧 sigma 数据：锚定
      sigma_anchor_pos_ = sigma_latest_pos_;
      sigma_anchor_ori_ = sigma_latest_ori_;
      franka_anchor_pos_ = Eigen::Vector3d(axis_[0].pos, axis_[1].pos, axis_[2].pos);
      franka_anchor_ori_ = cmd_ori_;
      anchored_ = true;
      RCLCPP_INFO(get_node()->get_logger(), "Anchored. Sigma at [%.3f, %.3f, %.3f]",
                  sigma_latest_pos_.x(), sigma_latest_pos_.y(), sigma_latest_pos_.z());
    }
  }

  if (anchored_) {
    // 计算增量（各轴独立缩放，含方向翻转）
    Eigen::Vector3d delta_sigma = sigma_latest_pos_ - sigma_anchor_pos_;
    Eigen::Vector3d delta_pos(
        scale_x_ * delta_sigma.x(),
        scale_y_ * delta_sigma.y(),
        scale_z_ * delta_sigma.z());

    Eigen::Quaterniond delta_ori = sigma_latest_ori_ * sigma_anchor_ori_.conjugate();
    if (delta_ori.w() < 0.0) delta_ori.coeffs() *= -1.0;
    delta_ori.normalize();
    // 缩放姿态增量：slerp(identity, delta, scale_ori_)
    Eigen::AngleAxisd aa(delta_ori);
    double scaled_angle = aa.angle() * scale_ori_;
    Eigen::Quaterniond scaled_delta(Eigen::AngleAxisd(scaled_angle, aa.axis()));

    // 应用增量到锚点
    Eigen::Vector3d new_goal = franka_anchor_pos_ + delta_pos;
    Eigen::Quaterniond new_goal_ori = scaled_delta * franka_anchor_ori_;
    new_goal_ori.normalize();

    // 工作空间边界保护
    for (int i = 0; i < 3; i++) {
      if (new_goal(i) < ws_min_[i]) new_goal(i) = ws_min_[i];
      if (new_goal(i) > ws_max_[i]) new_goal(i) = ws_max_[i];
      goal_pos_[i] = new_goal(i);
    }
    goal_ori_ = new_goal_ori;
  }

  // 二阶滤波平滑追踪
  for (int i = 0; i < 3; i++) {
    stepAxis(axis_[i], goal_pos_[i]);
  }
  stepOrientation();

  Eigen::Vector3d cmd_pos(axis_[0].pos, axis_[1].pos, axis_[2].pos);

  if (franka_cartesian_pose_->setCommand(cmd_ori_, cmd_pos)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_FATAL(get_node()->get_logger(), "Set command failed.");
    return controller_interface::return_type::ERROR;
  }
}

CallbackReturn CartesianPoseExampleController::on_init() {
  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated_));

  wn_pos_ = 5.0;
  zeta_pos_ = 1.2;
  max_vel_ = 0.20;
  max_acc_ = 1.5;
  max_jerk_ = 30.0;

  wn_ori_ = 5.0;
  zeta_ori_ = 1.2;
  max_ang_vel_ = 0.5;
  max_ang_acc_ = 3.0;
  max_ang_jerk_ = 100.0;

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

  auto future_result = client->async_send_request(request);
  future_result.wait_for(robot_utils::time_out);

  auto success = future_result.get();
  if (!success) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
  }

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  sigma_pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/sigma1/pose", rclcpp::SensorDataQoS(),
      std::bind(&CartesianPoseExampleController::sigmaPosCb, this, std::placeholders::_1));

  sigma_button_sub_ = get_node()->create_subscription<sensor_msgs::msg::Joy>(
      "/sigma1/buttons", rclcpp::SensorDataQoS(),
      std::bind(&CartesianPoseExampleController::sigmaButtonCb, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(),
              "Incremental teleop controller. scale=[%.1f, %.1f, %.1f] wn=%.1f max_v=%.2f",
              scale_x_, scale_y_, scale_z_, wn_pos_, max_vel_);
  RCLCPP_INFO(get_node()->get_logger(),
              "Subscribing to /sigma1/pose and /sigma1/buttons");

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  init_flag_ = true;
  franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseExampleController,
                       controller_interface::ControllerInterface)