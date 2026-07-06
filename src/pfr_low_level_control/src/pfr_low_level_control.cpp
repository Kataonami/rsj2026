#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/config.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pfr_low_level_control/pfr_low_level_control.hpp"

namespace
{
constexpr std::array<const char *, 3> kInputJointNames = {
  "joint1_R", "joint2_R", "joint3_R"};
constexpr std::array<const char *, 3> kModelJointNames = {
  "joint1_R", "joint2_R", "joint3_R"};
constexpr std::size_t kPlanarBaseDof = 3;
constexpr std::size_t kRightArmDof = 3;
constexpr std::size_t kGeneralizedDof = kPlanarBaseDof + kRightArmDof;

double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double quaternionYaw(const geometry_msgs::msg::Quaternion & quaternion_msg)
{
  Eigen::Quaterniond quaternion(
    quaternion_msg.w, quaternion_msg.x,
    quaternion_msg.y, quaternion_msg.z);
  if (quaternion.norm() < 1.0e-9) {
    return 0.0;
  }
  quaternion.normalize();
  const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

void setPlanarFloatingBaseConfiguration(
  const geometry_msgs::msg::Pose & base_pose,
  Eigen::VectorXd & q,
  Eigen::Index base_q_index)
{
  const double yaw = quaternionYaw(base_pose.orientation);
  q[base_q_index + 0] = base_pose.position.x;
  q[base_q_index + 1] = base_pose.position.y;
  q[base_q_index + 2] = 0.0;
  q[base_q_index + 3] = 0.0;
  q[base_q_index + 4] = 0.0;
  q[base_q_index + 5] = std::sin(0.5 * yaw);
  q[base_q_index + 6] = std::cos(0.5 * yaw);
}

bool solvePlanarPositionIk(
  const geometry_msgs::msg::Pose & desired_pose,
  const Eigen::Vector3d & current_q,
  const Eigen::Vector2d & first_joint_position,
  double first_link_length,
  double second_link_length,
  double first_link_angle,
  double second_link_angle,
  const Eigen::Vector3d & joint_axis_signs,
  const Eigen::Vector3d & lower_limits,
  const Eigen::Vector3d & upper_limits,
  Eigen::Vector3d & desired_q)
{
  const double target_x = desired_pose.position.x - first_joint_position.x();
  const double target_y = desired_pose.position.y - first_joint_position.y();
  const double radius_squared = target_x * target_x + target_y * target_y;
  const double cosine_elbow =
    (radius_squared - first_link_length * first_link_length -
    second_link_length * second_link_length) /
    (2.0 * first_link_length * second_link_length);
  if (cosine_elbow < -1.0 - 1.0e-9 || cosine_elbow > 1.0 + 1.0e-9) {
    return false;
  }

  const double bounded_cosine_elbow = std::clamp(cosine_elbow, -1.0, 1.0);
  const double elbow_magnitude = std::acos(bounded_cosine_elbow);
  const double desired_yaw = quaternionYaw(desired_pose.orientation);
  double best_distance = std::numeric_limits<double>::infinity();
  bool solution_found = false;

  for (const double elbow_angle : {elbow_magnitude, -elbow_magnitude}) {
    const double first_link_world_angle =
      std::atan2(target_y, target_x) -
      std::atan2(
      second_link_length * std::sin(elbow_angle),
      first_link_length + second_link_length * std::cos(elbow_angle));

    // For each planar joint, theta = theta_zero + axis_sign * q.  The right-arm
    // joints in the common URDF rotate about -Z, so their axis signs are -1.
    Eigen::Vector3d candidate;
    candidate[0] = wrapAngle(
      (first_link_world_angle - first_link_angle) / joint_axis_signs[0]);
    candidate[1] = wrapAngle(
      (elbow_angle - second_link_angle + first_link_angle) / joint_axis_signs[1]);
    candidate[2] = wrapAngle(
      (desired_yaw - joint_axis_signs[0] * candidate[0] -
      joint_axis_signs[1] * candidate[1]) / joint_axis_signs[2]);

    bool within_limits = true;
    for (Eigen::Index index = 0; index < candidate.size(); ++index) {
      within_limits = within_limits &&
        candidate[index] >= lower_limits[index] - 1.0e-9 &&
        candidate[index] <= upper_limits[index] + 1.0e-9;
    }
    if (!within_limits) {
      continue;
    }

    double distance = 0.0;
    for (Eigen::Index index = 0; index < candidate.size(); ++index) {
      distance += std::pow(wrapAngle(candidate[index] - current_q[index]), 2);
    }
    // Keep the first (positive-elbow) branch when both solutions are
    // numerically equidistant from a straight-arm pose.
    if (!solution_found || distance < best_distance - 1.0e-12) {
      best_distance = distance;
      desired_q = candidate;
      solution_found = true;
    }
  }

  return solution_found;
}
}  // namespace

struct PfrLowLevelControl::ControllerState
{
  pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> data;
  Eigen::VectorXd q;
  Eigen::Index base_q_index{-1};
  std::array<Eigen::Index, 3> base_v_indices{};
  std::array<Eigen::Index, 3> q_indices{};
  std::array<Eigen::Index, 3> v_indices{};
  std::array<pinocchio::JointIndex, 3> joint_ids{};
  std::array<bool, 3> joint_received{};
  pinocchio::FrameIndex ee_frame_id{0};
  Eigen::Vector2d first_joint_position{Eigen::Vector2d::Zero()};
  Eigen::Vector3d lower_limits{Eigen::Vector3d::Zero()};
  Eigen::Vector3d upper_limits{Eigen::Vector3d::Zero()};
  Eigen::Vector3d joint_axis_signs{Eigen::Vector3d::Ones()};
  double first_link_length{0.0};
  double second_link_length{0.0};
  double first_link_angle{0.0};
  double second_link_angle{0.0};

  geometry_msgs::msg::PoseStamped pose_reference;
  geometry_msgs::msg::TwistStamped twist_reference;
  rclcpp::Time last_base_pose_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_joint_state_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pose_reference_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_twist_reference_time{0, 0, RCL_ROS_TIME};
  bool base_pose_received{false};
  bool pose_reference_received{false};
  bool twist_reference_received{false};
  bool command_active{false};

  std::string control_mode{"generalized_jacobian"};
  double joint_position_gain{2.0};
  double task_space_gain{2.0};
  double damping{0.05};
  double max_base_linear_velocity{0.5};
  double max_base_yaw_velocity{1.0};
  double max_joint_velocity{2.0};
  double command_timeout_s{0.25};
  double control_rate_hz{50.0};
  std::string base_frame_id{"world"};
  std::string base_pose_topic{"/optitrack/base_pose"};
};

PfrLowLevelControl::PfrLowLevelControl()
: Node("pfr_low_level_control")
{
  controller_ = std::make_unique<ControllerState>();

  const auto default_urdf_path =
    ament_index_cpp::get_package_share_directory("PFR_Arm2") + "/PFR_Arm2.urdf";
  const auto urdf_path =
    this->declare_parameter<std::string>("urdf_path", default_urdf_path);
  const auto ee_frame =
    this->declare_parameter<std::string>("right_ee_frame", "joint3_R");
  const auto base_joint =
    this->declare_parameter<std::string>("base_joint", "world_to_base");
  controller_->base_frame_id =
    this->declare_parameter<std::string>("base_frame_id", "world");
  controller_->base_pose_topic =
    this->declare_parameter<std::string>("base_pose_topic", "/optitrack/base_pose");
  controller_->control_mode =
    this->declare_parameter<std::string>("control_mode", "generalized_jacobian");
  controller_->joint_position_gain =
    this->declare_parameter<double>("joint_position_gain", 2.0);
  controller_->task_space_gain =
    this->declare_parameter<double>("task_space_gain", 2.0);
  controller_->damping =
    this->declare_parameter<double>("damping", 0.05);
  controller_->max_base_linear_velocity =
    this->declare_parameter<double>("max_base_linear_velocity", 0.5);
  controller_->max_base_yaw_velocity =
    this->declare_parameter<double>("max_base_yaw_velocity", 1.0);
  controller_->max_joint_velocity =
    this->declare_parameter<double>("max_joint_velocity", 2.0);
  controller_->command_timeout_s =
    this->declare_parameter<double>("command_timeout_s", 0.25);
  controller_->control_rate_hz =
    this->declare_parameter<double>("control_rate_hz", 50.0);

  if (controller_->control_mode != "analytic_ik" &&
    controller_->control_mode != "generalized_jacobian")
  {
    throw std::invalid_argument(
            "control_mode must be 'analytic_ik' or 'generalized_jacobian'.");
  }
  if (controller_->joint_position_gain <= 0.0 ||
    controller_->task_space_gain <= 0.0 ||
    controller_->damping <= 0.0 || controller_->max_base_linear_velocity <= 0.0 ||
    controller_->max_base_yaw_velocity <= 0.0 || controller_->max_joint_velocity <= 0.0 ||
    controller_->command_timeout_s <= 0.0 || controller_->control_rate_hz <= 0.0)
  {
    throw std::invalid_argument("Low-level controller parameters must be positive.");
  }

  try {
    pinocchio::urdf::buildModel(urdf_path, controller_->model);
    controller_->data = std::make_unique<pinocchio::Data>(controller_->model);
    controller_->q = pinocchio::neutral(controller_->model);

    if (!controller_->model.existJointName(base_joint)) {
      throw std::runtime_error("Floating base joint not found: " + base_joint);
    }
    const auto base_joint_id = controller_->model.getJointId(base_joint);
    const auto & base_joint_model = controller_->model.joints[base_joint_id];
    if (base_joint_model.nq() < 7 || base_joint_model.nv() < 6) {
      throw std::runtime_error("Expected a floating base joint with nq>=7 and nv>=6: " + base_joint);
    }
    controller_->base_q_index = base_joint_model.idx_q();
    controller_->base_v_indices = {
      base_joint_model.idx_v(),
      base_joint_model.idx_v() + 1,
      base_joint_model.idx_v() + 5};

    for (std::size_t index = 0; index < kModelJointNames.size(); ++index) {
      const std::string joint_name = kModelJointNames[index];
      if (!controller_->model.existJointName(joint_name)) {
        throw std::runtime_error("URDF joint not found: " + joint_name);
      }
      const auto joint_id = controller_->model.getJointId(joint_name);
      const auto & joint_model = controller_->model.joints[joint_id];
      if (joint_model.nq() != 1 || joint_model.nv() != 1) {
        throw std::runtime_error("Expected one-DoF URDF joint: " + joint_name);
      }
      controller_->q_indices[index] = joint_model.idx_q();
      controller_->v_indices[index] = joint_model.idx_v();
      controller_->joint_ids[index] = joint_id;
      controller_->lower_limits[index] =
        controller_->model.lowerPositionLimit[joint_model.idx_q()];
      controller_->upper_limits[index] =
        controller_->model.upperPositionLimit[joint_model.idx_q()];
    }

    const Eigen::Vector3d first_joint_translation =
      controller_->model.jointPlacements[controller_->joint_ids[0]].translation();
    const Eigen::Vector3d first_link_translation =
      controller_->model.jointPlacements[controller_->joint_ids[1]].translation();
    const Eigen::Vector3d second_link_translation =
      controller_->model.jointPlacements[controller_->joint_ids[2]].translation();
    controller_->first_joint_position = first_joint_translation.head<2>();
    controller_->first_link_length = first_link_translation.head<2>().norm();
    controller_->second_link_length = second_link_translation.head<2>().norm();
    controller_->first_link_angle = std::atan2(
      first_link_translation.y(), first_link_translation.x());
    controller_->second_link_angle = std::atan2(
      second_link_translation.y(), second_link_translation.x());
    if (controller_->first_link_length <= 0.0 || controller_->second_link_length <= 0.0) {
      throw std::runtime_error("Planar right-arm link length must be positive.");
    }

    if (!controller_->model.existBodyName(ee_frame)) {
      throw std::runtime_error("Right end-effector frame not found: " + ee_frame);
    }
    controller_->ee_frame_id = controller_->model.getBodyId(ee_frame);

    Eigen::Matrix<double, 6, Eigen::Dynamic> neutral_jacobian(
      6, controller_->model.nv);
    neutral_jacobian.setZero();
    pinocchio::computeFrameJacobian(
      controller_->model, *controller_->data, controller_->q,
      controller_->ee_frame_id, pinocchio::LOCAL_WORLD_ALIGNED,
      neutral_jacobian);
    for (std::size_t index = 0; index < controller_->v_indices.size(); ++index) {
      const double angular_z = neutral_jacobian(5, controller_->v_indices[index]);
      if (std::abs(angular_z) < 0.5) {
        throw std::runtime_error(
                "Right-arm joint axis is not aligned with the planar Z axis: " +
                std::string(kModelJointNames[index]));
      }
      controller_->joint_axis_signs[index] = std::copysign(1.0, angular_z);
    }
    RCLCPP_INFO(
      this->get_logger(), "Right-arm planar joint axis signs: [%.0f, %.0f, %.0f].",
      controller_->joint_axis_signs[0], controller_->joint_axis_signs[1],
      controller_->joint_axis_signs[2]);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      this->get_logger(), "Failed to initialize Pinocchio from '%s': %s",
      urdf_path.c_str(), error.what());
    throw;
  }

  joint_velocity_pub_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/dxl_velocity_controller/commands", 10);
  base_velocity_pub_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/base_velocity_cmd", 10);
  base_pose_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseStamped>(
    controller_->base_pose_topic, 10,
    std::bind(
      &PfrLowLevelControl::basePoseCallback, this,
      std::placeholders::_1));
  right_joint_state_sub_ =
    this->create_subscription<sensor_msgs::msg::JointState>(
    "/pfr_state_estimator/right/current_joint_states", 10,
    std::bind(
      &PfrLowLevelControl::rightJointStateCallback, this,
      std::placeholders::_1));
  right_pose_ref_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/high_level/right/ee_pose_ref", 10,
    std::bind(
      &PfrLowLevelControl::rightPoseReferenceCallback, this,
      std::placeholders::_1));
  right_twist_ref_sub_ =
    this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/high_level/right/ee_twist_ref", 10,
    std::bind(
      &PfrLowLevelControl::rightTwistReferenceCallback, this,
      std::placeholders::_1));

  const auto timer_period = std::chrono::duration<double>(
    1.0 / controller_->control_rate_hz);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&PfrLowLevelControl::controlTimerCallback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Right-arm controller started in '%s' mode with Pinocchio %s at %.1f Hz; base pose topic: '%s'.",
    controller_->control_mode.c_str(), PINOCCHIO_VERSION, controller_->control_rate_hz,
    controller_->base_pose_topic.c_str());
}

PfrLowLevelControl::~PfrLowLevelControl() = default;

void PfrLowLevelControl::basePoseCallback(const geometry_msgs::msg::PoseStamped & pose)
{
  if (!pose.header.frame_id.empty() && pose.header.frame_id != controller_->base_frame_id) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Base pose frame '%s' does not match expected frame '%s'.",
      pose.header.frame_id.c_str(), controller_->base_frame_id.c_str());
  }

  setPlanarFloatingBaseConfiguration(
    pose.pose, controller_->q, controller_->base_q_index);
  controller_->base_pose_received = true;
  controller_->last_base_pose_time = this->now();
}

void PfrLowLevelControl::rightJointStateCallback(
  const sensor_msgs::msg::JointState & joint_state)
{
  if (joint_state.position.size() != joint_state.name.size()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Ignored right joint state because name and position sizes differ.");
    return;
  }

  for (std::size_t message_index = 0; message_index < joint_state.name.size(); ++message_index) {
    for (std::size_t joint_index = 0; joint_index < kInputJointNames.size(); ++joint_index) {
      if (joint_state.name[message_index] == kInputJointNames[joint_index]) {
        controller_->q[controller_->q_indices[joint_index]] =
          joint_state.position[message_index];
        controller_->joint_received[joint_index] = true;
      }
    }
  }
  controller_->last_joint_state_time = this->now();
}

void PfrLowLevelControl::rightPoseReferenceCallback(
  const geometry_msgs::msg::PoseStamped & pose)
{
  controller_->pose_reference = pose;
  controller_->pose_reference_received = true;
  controller_->last_pose_reference_time = this->now();
}

void PfrLowLevelControl::rightTwistReferenceCallback(
  const geometry_msgs::msg::TwistStamped & twist)
{
  controller_->twist_reference = twist;
  controller_->twist_reference_received = true;
  controller_->last_twist_reference_time = this->now();
}

void PfrLowLevelControl::controlTimerCallback()
{
  const bool all_joints_received = std::all_of(
    controller_->joint_received.begin(), controller_->joint_received.end(),
    [](bool received) {return received;});
  if (!controller_->base_pose_received || !all_joints_received ||
    !controller_->pose_reference_received ||
    !controller_->twist_reference_received)
  {
    return;
  }

  const rclcpp::Time now = this->now();
  const bool input_is_stale =
    (now - controller_->last_base_pose_time).seconds() > controller_->command_timeout_s ||
    (now - controller_->last_joint_state_time).seconds() > controller_->command_timeout_s ||
    (now - controller_->last_pose_reference_time).seconds() > controller_->command_timeout_s ||
    (now - controller_->last_twist_reference_time).seconds() > controller_->command_timeout_s;
  if (input_is_stale) {
    if (controller_->command_active) {
      publishZeroVelocity();
      controller_->command_active = false;
      RCLCPP_WARN(this->get_logger(), "IK input timed out; commanded zero joint velocity.");
    }
    return;
  }

  if (!controller_->pose_reference.header.frame_id.empty() &&
    controller_->pose_reference.header.frame_id != controller_->base_frame_id)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Pose reference frame '%s' does not match expected frame '%s'.",
      controller_->pose_reference.header.frame_id.c_str(),
      controller_->base_frame_id.c_str());
    return;
  }

  pinocchio::forwardKinematics(
    controller_->model, *controller_->data, controller_->q);
  pinocchio::updateFramePlacements(controller_->model, *controller_->data);

  Eigen::Matrix<double, 6, Eigen::Dynamic> full_jacobian(
    6, controller_->model.nv);
  full_jacobian.setZero();
  pinocchio::computeFrameJacobian(
    controller_->model, *controller_->data, controller_->q,
    controller_->ee_frame_id, pinocchio::LOCAL_WORLD_ALIGNED,
    full_jacobian);

  Eigen::Matrix<double, 3, kGeneralizedDof> generalized_task_jacobian;
  for (std::size_t index = 0; index < controller_->base_v_indices.size(); ++index) {
    const Eigen::Index column = controller_->base_v_indices[index];
    generalized_task_jacobian(0, index) = full_jacobian(0, column);
    generalized_task_jacobian(1, index) = full_jacobian(1, column);
    generalized_task_jacobian(2, index) = full_jacobian(5, column);
  }
  for (std::size_t index = 0; index < controller_->v_indices.size(); ++index) {
    const Eigen::Index column = controller_->v_indices[index];
    const std::size_t generalized_index = kPlanarBaseDof + index;
    generalized_task_jacobian(0, generalized_index) = full_jacobian(0, column);
    generalized_task_jacobian(1, generalized_index) = full_jacobian(1, column);
    generalized_task_jacobian(2, generalized_index) = full_jacobian(5, column);
  }
  const Eigen::Matrix3d arm_task_jacobian =
    generalized_task_jacobian.block<3, kRightArmDof>(0, kPlanarBaseDof);

  Eigen::Vector3d desired_task_velocity;
  desired_task_velocity <<
    controller_->twist_reference.twist.linear.x,
    controller_->twist_reference.twist.linear.y,
    controller_->twist_reference.twist.angular.z;

  Eigen::Vector3d base_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d joint_velocity;
  if (controller_->control_mode == "generalized_jacobian") {
    const auto & current_ee_pose =
      controller_->data->oMf[controller_->ee_frame_id];
    const double current_yaw = std::atan2(
      current_ee_pose.rotation()(1, 0), current_ee_pose.rotation()(0, 0));
    const double desired_yaw = quaternionYaw(
      controller_->pose_reference.pose.orientation);
    Eigen::Vector3d task_error;
    task_error <<
      controller_->pose_reference.pose.position.x - current_ee_pose.translation().x(),
      controller_->pose_reference.pose.position.y - current_ee_pose.translation().y(),
      wrapAngle(desired_yaw - current_yaw);

    const Eigen::Matrix3d damped_task_matrix =
      generalized_task_jacobian * generalized_task_jacobian.transpose() +
      std::pow(controller_->damping, 2) * Eigen::Matrix3d::Identity();
    const Eigen::Vector3d commanded_task_velocity =
      desired_task_velocity + controller_->task_space_gain * task_error;
    const Eigen::Matrix<double, kGeneralizedDof, 1> generalized_velocity =
      generalized_task_jacobian.transpose() *
      damped_task_matrix.ldlt().solve(commanded_task_velocity);
    base_velocity = generalized_velocity.head<kPlanarBaseDof>();
    joint_velocity = generalized_velocity.tail<kRightArmDof>();
  } else {
    Eigen::Vector3d current_right_q;
    for (std::size_t index = 0; index < controller_->q_indices.size(); ++index) {
      current_right_q[index] = controller_->q[controller_->q_indices[index]];
    }
    Eigen::Vector3d desired_right_q;
    if (!solvePlanarPositionIk(
        controller_->pose_reference.pose, current_right_q,
        controller_->first_joint_position,
        controller_->first_link_length, controller_->second_link_length,
        controller_->first_link_angle, controller_->second_link_angle,
        controller_->joint_axis_signs,
        controller_->lower_limits, controller_->upper_limits,
        desired_right_q))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Right-hand pose reference is outside the reachable planar workspace.");
      if (controller_->command_active) {
        publishZeroVelocity();
        controller_->command_active = false;
      }
      return;
    }

    const Eigen::Matrix3d damped_task_matrix =
      arm_task_jacobian * arm_task_jacobian.transpose() +
      std::pow(controller_->damping, 2) * Eigen::Matrix3d::Identity();
    joint_velocity = arm_task_jacobian.transpose() *
      damped_task_matrix.ldlt().solve(desired_task_velocity);
    for (Eigen::Index index = 0; index < joint_velocity.size(); ++index) {
      joint_velocity[index] += controller_->joint_position_gain *
        wrapAngle(desired_right_q[index] - current_right_q[index]);
    }
  }

  if (!base_velocity.allFinite() || !joint_velocity.allFinite()) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Differential IK produced a non-finite command; commanding zero velocity.");
    publishZeroVelocity();
    controller_->command_active = false;
    return;
  }

  std_msgs::msg::Float64MultiArray base_command;
  base_command.data.resize(kPlanarBaseDof);
  base_command.data[0] = std::clamp(
    base_velocity[0], -controller_->max_base_linear_velocity,
    controller_->max_base_linear_velocity);
  base_command.data[1] = std::clamp(
    base_velocity[1], -controller_->max_base_linear_velocity,
    controller_->max_base_linear_velocity);
  base_command.data[2] = std::clamp(
    base_velocity[2], -controller_->max_base_yaw_velocity,
    controller_->max_base_yaw_velocity);
  base_velocity_pub_->publish(base_command);

  std_msgs::msg::Float64MultiArray command;
  command.data.resize(3);
  for (Eigen::Index index = 0; index < joint_velocity.size(); ++index) {
    command.data[static_cast<std::size_t>(index)] = std::clamp(
      joint_velocity[index], -controller_->max_joint_velocity,
      controller_->max_joint_velocity);
  }
  joint_velocity_pub_->publish(command);
  controller_->command_active = true;
}

void PfrLowLevelControl::publishZeroVelocity()
{
  std_msgs::msg::Float64MultiArray base_command;
  base_command.data = {0.0, 0.0, 0.0};
  base_velocity_pub_->publish(base_command);

  std_msgs::msg::Float64MultiArray command;
  command.data = {0.0, 0.0, 0.0};
  joint_velocity_pub_->publish(command);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PfrLowLevelControl>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
