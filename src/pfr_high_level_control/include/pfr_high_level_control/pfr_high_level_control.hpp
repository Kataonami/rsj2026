#ifndef PFR_HIGH_LEVEL_CONTROL__PFR_HIGH_LEVEL_CONTROL_HPP_
#define PFR_HIGH_LEVEL_CONTROL__PFR_HIGH_LEVEL_CONTROL_HPP_

#include <cstdint>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

class PfrHighLevelControl : public rclcpp::Node
{
public:
  PfrHighLevelControl();

private:
  static constexpr std::uint8_t kRightArmCircleCommand = 1;
  static constexpr std::uint8_t kLeftArmCircleCommand = 2;

  enum class TrajectoryPhase : std::uint8_t
  {
    kIdle,
    kMoveToReady,
    kCircle
  };

  struct CircleTrajectoryState
  {
    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::PoseStamped start_pose;
    rclcpp::Time start_time{0, 0, RCL_ROS_TIME};
    bool current_pose_received = false;
    TrajectoryPhase phase = TrajectoryPhase::kIdle;
  };

  void eeVelCommandCallback(const std_msgs::msg::Float64MultiArray & ee_vel_cmd);
  void circleTrajectoryCommandCallback(const std_msgs::msg::UInt8 & circle_cmd);
  void rightEePoseCallback(const geometry_msgs::msg::PoseStamped & pose);
  void leftEePoseCallback(const geometry_msgs::msg::PoseStamped & pose);
  void rightJointStateCallback(const sensor_msgs::msg::JointState & joint_state);
  void leftJointStateCallback(const sensor_msgs::msg::JointState & joint_state);
  void trajectoryTimerCallback();
  void startCircleTrajectory(CircleTrajectoryState & state, const char * arm_name);
  void publishTrajectoryReference(
    CircleTrajectoryState & state,
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pose_pub,
    const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr & twist_pub,
    const char * arm_name);
  void eeVelCommandPublisher();

  // pub
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ee_vel_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_ee_pose_ref_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr right_ee_twist_ref_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_ee_pose_ref_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr left_ee_twist_ref_pub_;

  // sub
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ee_vel_command_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr circle_trajectory_command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_ee_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_ee_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_joint_state_sub_;

  // timer
  rclcpp::TimerBase::SharedPtr trajectory_timer_;

  double speed_limit_ = 0.25; // Set the maximum velocity
  double ready_move_duration_s_ = 3.0;
  double right_ready_x_m_ = 0.0;
  double right_ready_y_m_ = -0.660114;
  double right_ready_z_m_ = 0.1975;
  double left_ready_x_m_ = 0.0;
  double left_ready_y_m_ = 0.660114;
  double left_ready_z_m_ = 0.1975;
  double circle_radius_m_ = 0.05;
  double circle_duration_s_ = 10.0;
  double circle_revolutions_ = 1.0;
  double trajectory_publish_rate_hz_ = 50.0;
  std_msgs::msg::Float64MultiArray ee_vel_cmd_;
  sensor_msgs::msg::JointState right_current_joint_state_;
  sensor_msgs::msg::JointState left_current_joint_state_;
  bool right_joint_state_received_ = false;
  bool left_joint_state_received_ = false;
  CircleTrajectoryState right_circle_state_;
  CircleTrajectoryState left_circle_state_;

};

#endif  // PFR_HIGH_LEVEL_CONTROL__PFR_HIGH_LEVEL_CONTROL_HPP_
