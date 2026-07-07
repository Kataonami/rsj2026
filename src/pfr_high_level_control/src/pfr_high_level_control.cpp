#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "pfr_high_level_control/pfr_high_level_control.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct TimeScaling
{
  double position;
  double velocity;
};

TimeScaling quinticTimeScaling(double elapsed_s, double duration_s)
{
  const double tau = std::clamp(elapsed_s / duration_s, 0.0, 1.0);
  const double tau2 = tau * tau;
  const double tau3 = tau2 * tau;
  const double tau4 = tau3 * tau;
  const double tau5 = tau4 * tau;
  return {
    10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5,
    (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4) / duration_s};
}

double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double quaternionYaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::atan2(
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1.0 - 2.0 *
    (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}
}

PfrHighLevelControl::PfrHighLevelControl()
: Node("pfr_high_level_control")
{
  std::cout << "PfrHighLevelControl class is established." << std::endl;

  // pub
  ee_vel_command_pub_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("/low_level/ee_vel_cmd", 10);
  right_ee_pose_ref_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/high_level/right/ee_pose_ref", 10);
  right_ee_twist_ref_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/high_level/right/ee_twist_ref", 10);
  left_ee_pose_ref_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/high_level/left/ee_pose_ref", 10);
  left_ee_twist_ref_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/high_level/left/ee_twist_ref", 10);

  ready_move_duration_s_ = this->declare_parameter<double>("ready_move_duration_s", 10.0);
  right_ready_x_m_ = this->declare_parameter<double>("right_ready_x_m", 0.0);
  right_ready_y_m_ = this->declare_parameter<double>("right_ready_y_m", -0.560114);
  right_ready_z_m_ = this->declare_parameter<double>("right_ready_z_m", 0.1975);
  left_ready_x_m_ = this->declare_parameter<double>("left_ready_x_m", 0.0);
  left_ready_y_m_ = this->declare_parameter<double>("left_ready_y_m", 0.560114);
  left_ready_z_m_ = this->declare_parameter<double>("left_ready_z_m", 0.1975);
  circle_radius_m_ = this->declare_parameter<double>("circle_radius_m", 0.1);
  circle_duration_s_ = this->declare_parameter<double>("circle_duration_s", 20.0);
  circle_revolutions_ = this->declare_parameter<double>("circle_revolutions", 1.0);
  trajectory_publish_rate_hz_ =
    this->declare_parameter<double>("trajectory_publish_rate_hz", 20.0);
  const auto base_pose_topic =
    this->declare_parameter<std::string>("base_pose_topic", "/optitrack/base_pose");

  if (ready_move_duration_s_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "ready_move_duration_s must be positive; using 10 s.");
    ready_move_duration_s_ = 10.0;
  }
  if (circle_radius_m_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "circle_radius_m must be positive; using 0.1 m.");
    circle_radius_m_ = 0.1;
  }
  if (circle_duration_s_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "circle_duration_s must be positive; using 20 s.");
    circle_duration_s_ = 20.0;
  }
  if (circle_revolutions_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "circle_revolutions must be positive; using 1.");
    circle_revolutions_ = 1.0;
  }
  if (trajectory_publish_rate_hz_ <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(), "trajectory_publish_rate_hz must be positive; using 20 Hz.");
    trajectory_publish_rate_hz_ = 20.0;
  }

  // sub
  ee_vel_command_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/ee_vel_cmd", 10, std::bind(&PfrHighLevelControl::eeVelCommandCallback, this, std::placeholders::_1));
  circle_trajectory_command_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
    "/circle_trajectory_cmd", 10,
    std::bind(
      &PfrHighLevelControl::circleTrajectoryCommandCallback, this,
      std::placeholders::_1));
  right_ee_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/current_ee_pose_R", 10,
    std::bind(&PfrHighLevelControl::rightEePoseCallback, this, std::placeholders::_1));
  left_ee_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/current_ee_pose_L", 10,
    std::bind(&PfrHighLevelControl::leftEePoseCallback, this, std::placeholders::_1));
  right_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/pfr_state_estimator/right/current_joint_states", 10,
    std::bind(
      &PfrHighLevelControl::rightJointStateCallback, this,
      std::placeholders::_1));
  left_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/pfr_state_estimator/left/current_joint_states", 10,
    std::bind(
      &PfrHighLevelControl::leftJointStateCallback, this,
      std::placeholders::_1));

  base_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    base_pose_topic, 10,
    std::bind(&PfrHighLevelControl::basePoseCallback, this, std::placeholders::_1));

  // timer
  const auto timer_period = std::chrono::duration<double>(1.0 / trajectory_publish_rate_hz_);
  trajectory_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&PfrHighLevelControl::trajectoryTimerCallback, this));

}

void PfrHighLevelControl::eeVelCommandCallback(const std_msgs::msg::Float64MultiArray & ee_vel_cmd)
{
  ee_vel_cmd_ = ee_vel_cmd;
  eeVelCommandPublisher();
}

void PfrHighLevelControl::circleTrajectoryCommandCallback(
  const std_msgs::msg::UInt8 & circle_cmd)
{
  if (circle_cmd.data != kRightArmCircleCommand &&
    circle_cmd.data != kLeftArmCircleCommand)
  {
    RCLCPP_WARN(
      this->get_logger(), "Unknown circle trajectory arm command: %u", circle_cmd.data);
    return;
  }

  if (circle_cmd.data == kRightArmCircleCommand) {
    startCircleTrajectory(right_circle_state_, "right");
  } else {
    startCircleTrajectory(left_circle_state_, "left");
  }
}

void PfrHighLevelControl::rightEePoseCallback(
  const geometry_msgs::msg::PoseStamped & pose)
{
  right_circle_state_.current_pose = pose;
  right_circle_state_.current_pose_received = true;
}

void PfrHighLevelControl::leftEePoseCallback(
  const geometry_msgs::msg::PoseStamped & pose)
{
  left_circle_state_.current_pose = pose;
  left_circle_state_.current_pose_received = true;
}

void PfrHighLevelControl::basePoseCallback(
  const geometry_msgs::msg::PoseStamped & pose)
{
  base_pose_ = pose;
  if (!base_pose_received_) {
    RCLCPP_INFO(
      this->get_logger(), "Received OptiTrack base pose on frame '%s'.",
      pose.header.frame_id.c_str());
    base_pose_received_ = true;
  }
}

void PfrHighLevelControl::rightJointStateCallback(
  const sensor_msgs::msg::JointState & joint_state)
{
  right_current_joint_state_ = joint_state;
  if (!right_joint_state_received_) {
    RCLCPP_INFO(
      this->get_logger(), "Received right-arm joint state from State Estimator.");
    right_joint_state_received_ = true;
  }
}

void PfrHighLevelControl::leftJointStateCallback(
  const sensor_msgs::msg::JointState & joint_state)
{
  left_current_joint_state_ = joint_state;
  if (!left_joint_state_received_) {
    RCLCPP_INFO(
      this->get_logger(), "Received left-arm joint state from State Estimator.");
    left_joint_state_received_ = true;
  }
}

void PfrHighLevelControl::startCircleTrajectory(
  CircleTrajectoryState & state, const char * arm_name)
{
  if (!state.current_pose_received) {
    RCLCPP_WARN(
      this->get_logger(),
      "%s-hand circle trajectory was not started: current end-effector pose is unavailable.",
      arm_name);
    return;
  }

  state.start_pose = state.current_pose;
  if (state.start_pose.header.frame_id.empty()) {
    state.start_pose.header.frame_id = "world";
  }
  state.start_time = this->now();
  state.phase = TrajectoryPhase::kMoveToReady;

  const bool is_right_arm = std::string(arm_name) == "right";
  const double ready_x = is_right_arm ? right_ready_x_m_ : left_ready_x_m_;
  const double ready_y = is_right_arm ? right_ready_y_m_ : left_ready_y_m_;
  const double ready_z = is_right_arm ? right_ready_z_m_ : left_ready_z_m_;

  RCLCPP_INFO(
    this->get_logger(),
    "%s-hand preparation started: current=(%.4f, %.4f, %.4f), "
    "ready=(%.4f, %.4f, %.4f), duration=%.3f s.",
    arm_name,
    state.start_pose.pose.position.x,
    state.start_pose.pose.position.y,
    state.start_pose.pose.position.z,
    ready_x, ready_y, ready_z, ready_move_duration_s_);
}

void PfrHighLevelControl::trajectoryTimerCallback()
{
  publishTrajectoryReference(
    right_circle_state_, right_ee_pose_ref_pub_, right_ee_twist_ref_pub_, "right");
  publishTrajectoryReference(
    left_circle_state_, left_ee_pose_ref_pub_, left_ee_twist_ref_pub_, "left");
}

void PfrHighLevelControl::publishTrajectoryReference(
  CircleTrajectoryState & state,
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pose_pub,
  const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr & twist_pub,
  const char * arm_name)
{
  if (state.phase == TrajectoryPhase::kIdle) {
    return;
  }

  const rclcpp::Time stamp = this->now();
  const double elapsed_s = (stamp - state.start_time).seconds();

  geometry_msgs::msg::PoseStamped pose_ref;
  pose_ref.header.stamp = stamp;
  pose_ref.header.frame_id = state.start_pose.header.frame_id;
  geometry_msgs::msg::TwistStamped twist_ref;
  twist_ref.header = pose_ref.header;

  if (state.phase == TrajectoryPhase::kMoveToReady) {
    const TimeScaling scaling = quinticTimeScaling(elapsed_s, ready_move_duration_s_);
    const bool is_right_arm = std::string(arm_name) == "right";
    const double ready_x = is_right_arm ? right_ready_x_m_ : left_ready_x_m_;
    const double ready_y = is_right_arm ? right_ready_y_m_ : left_ready_y_m_;
    const double ready_z = is_right_arm ? right_ready_z_m_ : left_ready_z_m_;
    const double delta_x = ready_x - state.start_pose.pose.position.x;
    const double delta_y = ready_y - state.start_pose.pose.position.y;
    const double delta_z = ready_z - state.start_pose.pose.position.z;

    pose_ref.pose.position.x = state.start_pose.pose.position.x + scaling.position * delta_x;
    pose_ref.pose.position.y = state.start_pose.pose.position.y + scaling.position * delta_y;
    pose_ref.pose.position.z = state.start_pose.pose.position.z + scaling.position * delta_z;
    twist_ref.twist.linear.x = scaling.velocity * delta_x;
    twist_ref.twist.linear.y = scaling.velocity * delta_y;
    twist_ref.twist.linear.z = scaling.velocity * delta_z;

    const double start_yaw = quaternionYaw(state.start_pose.pose.orientation);
    const double yaw_error = wrapAngle(-start_yaw);
    const double yaw_ref = start_yaw + scaling.position * yaw_error;
    pose_ref.pose.orientation.z = std::sin(0.5 * yaw_ref);
    pose_ref.pose.orientation.w = std::cos(0.5 * yaw_ref);
    twist_ref.twist.angular.z = scaling.velocity * yaw_error;

    if (elapsed_s >= ready_move_duration_s_) {
      pose_ref.pose.position.x = ready_x;
      pose_ref.pose.position.y = ready_y;
      pose_ref.pose.position.z = ready_z;
      pose_ref.pose.orientation.z = 0.0;
      pose_ref.pose.orientation.w = 1.0;
      twist_ref.twist.linear.x = 0.0;
      twist_ref.twist.linear.y = 0.0;
      twist_ref.twist.linear.z = 0.0;
      twist_ref.twist.angular.z = 0.0;

      state.start_pose = pose_ref;
      state.start_time = stamp;
      state.phase = TrajectoryPhase::kCircle;
      RCLCPP_INFO(
        this->get_logger(),
        "%s hand reached the ready pose; circle trajectory started.", arm_name);
    }
  } else {
    const TimeScaling scaling = quinticTimeScaling(elapsed_s, circle_duration_s_);
    const double total_angle = 2.0 * kPi * circle_revolutions_;
    const double phase = total_angle * scaling.position;
    const double phase_rate = total_angle * scaling.velocity;

    pose_ref.pose.position.x =
      state.start_pose.pose.position.x + circle_radius_m_ * (std::cos(phase) - 1.0);
    pose_ref.pose.position.y =
      state.start_pose.pose.position.y + circle_radius_m_ * std::sin(phase);
    pose_ref.pose.position.z = state.start_pose.pose.position.z;
    // Identity orientation keeps the end-effector frame pointed along world +X.
    pose_ref.pose.orientation.w = 1.0;
    twist_ref.twist.linear.x = -circle_radius_m_ * std::sin(phase) * phase_rate;
    twist_ref.twist.linear.y = circle_radius_m_ * std::cos(phase) * phase_rate;

    if (elapsed_s >= circle_duration_s_) {
      pose_ref.pose.position = state.start_pose.pose.position;
      twist_ref.twist.linear.x = 0.0;
      twist_ref.twist.linear.y = 0.0;
      state.phase = TrajectoryPhase::kIdle;
      RCLCPP_INFO(this->get_logger(), "%s-hand circle trajectory completed.", arm_name);
    }
  }

  pose_pub->publish(pose_ref);
  twist_pub->publish(twist_ref);
}

void PfrHighLevelControl::eeVelCommandPublisher()
{
  // Forward the manual Cartesian velocity command to the low-level controller.
  ee_vel_command_pub_->publish(ee_vel_cmd_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PfrHighLevelControl>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();

  return 0;
}
