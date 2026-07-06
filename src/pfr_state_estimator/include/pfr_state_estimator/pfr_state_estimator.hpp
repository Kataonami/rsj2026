#ifndef PFR_STATE_ESTIMATOR__PFR_STATE_ESTIMATOR_HPP_
#define PFR_STATE_ESTIMATOR__PFR_STATE_ESTIMATOR_HPP_

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class PfrStateEstimator : public rclcpp::Node
{
public:
  PfrStateEstimator();
  ~PfrStateEstimator() override;

private:
  void jointStateCallback(const sensor_msgs::msg::JointState & joint_state_msg);

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    right_current_joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    left_current_joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_ee_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_ee_pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoder_joint_state_sub_;

  struct KinematicsState;
  std::unique_ptr<KinematicsState> kinematics_;
};

#endif  // PFR_STATE_ESTIMATOR__PFR_STATE_ESTIMATOR_HPP_
