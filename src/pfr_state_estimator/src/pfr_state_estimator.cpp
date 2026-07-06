#include <algorithm>
#include <array>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/config.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pfr_state_estimator/pfr_state_estimator.hpp"

namespace
{
constexpr std::array<const char *, 3> kRightJointNames = {
  "joint1", "joint2", "joint3"};
constexpr std::array<const char *, 3> kLeftJointNames = {
  "joint4", "joint5", "joint6"};
constexpr std::array<const char *, 3> kRightModelJointNames = {
  "Joint1_R", "Joint2_R", "Joint3_R"};
constexpr std::array<const char *, 3> kLeftModelJointNames = {
  "Joint1_L", "Joint2_L", "Joint3_L"};

template<std::size_t N>
sensor_msgs::msg::JointState selectJointStates(
  const sensor_msgs::msg::JointState & input,
  const std::array<const char *, N> & selected_names)
{
  sensor_msgs::msg::JointState output;
  output.header = input.header;

  for (const char * selected_name : selected_names) {
    const auto joint_it =
      std::find(input.name.begin(), input.name.end(), selected_name);
    if (joint_it == input.name.end()) {
      continue;
    }

    const auto index = static_cast<std::size_t>(
      std::distance(input.name.begin(), joint_it));
    output.name.push_back(*joint_it);

    if (!input.position.empty()) {
      output.position.push_back(input.position[index]);
    }
    if (!input.velocity.empty()) {
      output.velocity.push_back(input.velocity[index]);
    }
    if (!input.effort.empty()) {
      output.effort.push_back(input.effort[index]);
    }
  }

  return output;
}

bool fieldSizesAreValid(const sensor_msgs::msg::JointState & joint_state)
{
  const auto joint_count = joint_state.name.size();
  return
    (joint_state.position.empty() || joint_state.position.size() == joint_count) &&
    (joint_state.velocity.empty() || joint_state.velocity.size() == joint_count) &&
    (joint_state.effort.empty() || joint_state.effort.size() == joint_count);
}

bool allJointsReceived(const std::array<bool, 3> & received)
{
  return std::all_of(received.begin(), received.end(), [](bool value) {return value;});
}

geometry_msgs::msg::PoseStamped makePoseStamped(
  const pinocchio::SE3 & placement,
  const std_msgs::msg::Header & input_header,
  const std::string & base_frame_id)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = input_header;
  pose.header.frame_id = base_frame_id;
  pose.pose.position.x = placement.translation().x();
  pose.pose.position.y = placement.translation().y();
  pose.pose.position.z = placement.translation().z();

  Eigen::Quaterniond quaternion(placement.rotation());
  quaternion.normalize();
  pose.pose.orientation.x = quaternion.x();
  pose.pose.orientation.y = quaternion.y();
  pose.pose.orientation.z = quaternion.z();
  pose.pose.orientation.w = quaternion.w();
  return pose;
}
}  // namespace

struct PfrStateEstimator::KinematicsState
{
  pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> data;
  Eigen::VectorXd q;
  std::array<Eigen::Index, 3> right_q_indices{};
  std::array<Eigen::Index, 3> left_q_indices{};
  std::array<bool, 3> right_joint_received{};
  std::array<bool, 3> left_joint_received{};
  pinocchio::FrameIndex right_ee_frame_id{0};
  pinocchio::FrameIndex left_ee_frame_id{0};
  std::string base_frame_id{"base_link"};
};

PfrStateEstimator::PfrStateEstimator()
: Node("pfr_state_estimator")
{
  RCLCPP_INFO(this->get_logger(), "PFR state estimator started.");

  kinematics_ = std::make_unique<KinematicsState>();

  const auto default_urdf_path =
    ament_index_cpp::get_package_share_directory("PFR_Arm2") + "/PFR_Arm2.urdf";
  const auto urdf_path =
    this->declare_parameter<std::string>("urdf_path", default_urdf_path);
  const auto right_ee_frame =
    this->declare_parameter<std::string>("right_ee_frame", "Joint3_R");
  const auto left_ee_frame =
    this->declare_parameter<std::string>("left_ee_frame", "Joint3_L");
  kinematics_->base_frame_id =
    this->declare_parameter<std::string>("base_frame_id", "base_link");

  try {
    pinocchio::urdf::buildModel(urdf_path, kinematics_->model);
    kinematics_->data = std::make_unique<pinocchio::Data>(kinematics_->model);
    kinematics_->q = pinocchio::neutral(kinematics_->model);

    const auto configure_joint_indices =
      [this](
      const std::array<const char *, 3> & model_joint_names,
      std::array<Eigen::Index, 3> & q_indices)
      {
        for (std::size_t index = 0; index < model_joint_names.size(); ++index) {
          const std::string joint_name = model_joint_names[index];
          if (!kinematics_->model.existJointName(joint_name)) {
            throw std::runtime_error("URDF joint not found: " + joint_name);
          }

          const auto joint_id = kinematics_->model.getJointId(joint_name);
          const auto & joint_model = kinematics_->model.joints[joint_id];
          if (joint_model.nq() != 1) {
            throw std::runtime_error("Expected one-DoF URDF joint: " + joint_name);
          }
          q_indices[index] = joint_model.idx_q();
        }
      };

    configure_joint_indices(kRightModelJointNames, kinematics_->right_q_indices);
    configure_joint_indices(kLeftModelJointNames, kinematics_->left_q_indices);

    if (!kinematics_->model.existBodyName(right_ee_frame)) {
      throw std::runtime_error("Right end-effector frame not found: " + right_ee_frame);
    }
    if (!kinematics_->model.existBodyName(left_ee_frame)) {
      throw std::runtime_error("Left end-effector frame not found: " + left_ee_frame);
    }
    kinematics_->right_ee_frame_id = kinematics_->model.getBodyId(right_ee_frame);
    kinematics_->left_ee_frame_id = kinematics_->model.getBodyId(left_ee_frame);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      this->get_logger(), "Failed to initialize Pinocchio from '%s': %s",
      urdf_path.c_str(), error.what());
    throw;
  }

  // pub
  right_current_joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>(
    "/pfr_state_estimator/right/current_joint_states", 10);
  left_current_joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>(
    "/pfr_state_estimator/left/current_joint_states", 10);
  right_ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/current_ee_pose_R", 10);
  left_ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/current_ee_pose_L", 10);

  // sub
  encoder_joint_state_sub_ =
    this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(
      &PfrStateEstimator::jointStateCallback, this,
      std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "Loaded Pinocchio %s model from '%s' (nq=%d, nv=%d).",
    PINOCCHIO_VERSION, urdf_path.c_str(), kinematics_->model.nq,
    kinematics_->model.nv);
}

PfrStateEstimator::~PfrStateEstimator() = default;

void PfrStateEstimator::jointStateCallback(
  const sensor_msgs::msg::JointState & joint_state_msg)
{
  if (!fieldSizesAreValid(joint_state_msg)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Ignored /joint_states because name and data array sizes do not match.");
    return;
  }

  const auto right_joint_states =
    selectJointStates(joint_state_msg, kRightJointNames);
  if (!right_joint_states.name.empty()) {
    right_current_joint_state_pub_->publish(right_joint_states);
  }

  const auto left_joint_states =
    selectJointStates(joint_state_msg, kLeftJointNames);
  if (!left_joint_states.name.empty()) {
    left_current_joint_state_pub_->publish(left_joint_states);
  }

  if (joint_state_msg.position.empty()) {
    return;
  }

  bool right_updated = false;
  bool left_updated = false;
  for (std::size_t message_index = 0;
    message_index < joint_state_msg.name.size(); ++message_index)
  {
    for (std::size_t joint_index = 0; joint_index < kRightJointNames.size(); ++joint_index) {
      if (joint_state_msg.name[message_index] == kRightJointNames[joint_index]) {
        kinematics_->q[kinematics_->right_q_indices[joint_index]] =
          joint_state_msg.position[message_index];
        kinematics_->right_joint_received[joint_index] = true;
        right_updated = true;
      }
      if (joint_state_msg.name[message_index] == kLeftJointNames[joint_index]) {
        kinematics_->q[kinematics_->left_q_indices[joint_index]] =
          joint_state_msg.position[message_index];
        kinematics_->left_joint_received[joint_index] = true;
        left_updated = true;
      }
    }
  }

  const bool right_ready = right_updated && allJointsReceived(
    kinematics_->right_joint_received);
  const bool left_ready = left_updated && allJointsReceived(
    kinematics_->left_joint_received);
  if (!right_ready && !left_ready) {
    return;
  }

  pinocchio::forwardKinematics(
    kinematics_->model, *kinematics_->data, kinematics_->q);
  pinocchio::updateFramePlacements(kinematics_->model, *kinematics_->data);

  if (right_ready) {
    right_ee_pose_pub_->publish(makePoseStamped(
      kinematics_->data->oMf[kinematics_->right_ee_frame_id],
      joint_state_msg.header, kinematics_->base_frame_id));
  }
  if (left_ready) {
    left_ee_pose_pub_->publish(makePoseStamped(
      kinematics_->data->oMf[kinematics_->left_ee_frame_id],
      joint_state_msg.header, kinematics_->base_frame_id));
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PfrStateEstimator>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();

  return 0;
}
