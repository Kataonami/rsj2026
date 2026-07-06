#ifndef PFR_LOW_LEVEL_CONTROL__PFR_LOW_LEVEL_CONTROL_HPP_
#define PFR_LOW_LEVEL_CONTROL__PFR_LOW_LEVEL_CONTROL_HPP_

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class PfrLowLevelControl : public rclcpp::Node
{
public:
  PfrLowLevelControl();
  ~PfrLowLevelControl() override;

private:
  void rightJointStateCallback(const sensor_msgs::msg::JointState & joint_state);
  void rightPoseReferenceCallback(const geometry_msgs::msg::PoseStamped & pose);
  void rightTwistReferenceCallback(const geometry_msgs::msg::TwistStamped & twist);
  void controlTimerCallback();
  void publishZeroVelocity();

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_ref_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr right_twist_ref_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  struct ControllerState;
  std::unique_ptr<ControllerState> controller_;
};

#endif  // PFR_LOW_LEVEL_CONTROL__PFR_LOW_LEVEL_CONTROL_HPP_
