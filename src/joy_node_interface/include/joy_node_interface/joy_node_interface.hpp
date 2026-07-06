#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

class CommandInterface : public rclcpp::Node
{
public:
  CommandInterface();
private:
  static constexpr std::uint8_t kRightArmCircleCommand = 1;
  static constexpr std::uint8_t kLeftArmCircleCommand = 2;

  void joyCallback(const sensor_msgs::msg::Joy & joy_cmd);
  void timerCallback();

  // pub
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ee_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr circle_trajectory_cmd_pub_;

  // sub
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Joy input_joy_cmd;
  int square_button_index_;
  int circle_button_index_;
  int option_button_index_;
  bool right_circle_combo_was_pressed_ = false;
  bool left_circle_combo_was_pressed_ = false;

};
