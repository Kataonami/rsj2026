#include "joy_node_interface/joy_node_interface.hpp"

CommandInterface::CommandInterface()
: Node("joy_node_interface")
{
  std::cout << "CommandInterface class is established." << std::endl;

  // pub
  ee_vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ee_vel_cmd", 10);
  circle_trajectory_cmd_pub_ =
    this->create_publisher<std_msgs::msg::UInt8>("/circle_trajectory_cmd", 10);

  square_button_index_ = this->declare_parameter<int>("square_button_index", 3);
  circle_button_index_ = this->declare_parameter<int>("circle_button_index", 1);
  option_button_index_ = this->declare_parameter<int>("option_button_index", 9);

  // sub
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10, std::bind(&CommandInterface::joyCallback, this, std::placeholders::_1));

  // timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&CommandInterface::timerCallback, this));

  input_joy_cmd.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  input_joy_cmd.axes = {0, 0, 0, 0, 0, 0, 0, 0};

}

void CommandInterface::joyCallback(const sensor_msgs::msg::Joy & joy_cmd)
{
  input_joy_cmd = joy_cmd;
}

void CommandInterface::timerCallback()
{
  const auto button_is_pressed = [this](int index) {
      return
        index >= 0 &&
        static_cast<std::size_t>(index) < input_joy_cmd.buttons.size() &&
        input_joy_cmd.buttons[static_cast<std::size_t>(index)] == 1;
    };

  const bool square_button_pressed = button_is_pressed(square_button_index_);
  const bool circle_button_pressed = button_is_pressed(circle_button_index_);
  const bool option_button_pressed = button_is_pressed(option_button_index_);
  const bool right_circle_combo_pressed =
    square_button_pressed && option_button_pressed && !circle_button_pressed;
  const bool left_circle_combo_pressed =
    circle_button_pressed && option_button_pressed && !square_button_pressed;

  // Square + Option starts the right-hand circular trajectory.
  if (right_circle_combo_pressed && !right_circle_combo_was_pressed_) {
    std_msgs::msg::UInt8 circle_cmd;
    circle_cmd.data = kRightArmCircleCommand;
    circle_trajectory_cmd_pub_->publish(circle_cmd);
    RCLCPP_INFO(this->get_logger(), "Right-hand circle trajectory command sent.");
  }

  // Circle + Option starts the left-hand circular trajectory.
  if (left_circle_combo_pressed && !left_circle_combo_was_pressed_) {
    std_msgs::msg::UInt8 circle_cmd;
    circle_cmd.data = kLeftArmCircleCommand;
    circle_trajectory_cmd_pub_->publish(circle_cmd);
    RCLCPP_INFO(this->get_logger(), "Left-hand circle trajectory command sent.");
  }

  right_circle_combo_was_pressed_ = right_circle_combo_pressed;
  left_circle_combo_was_pressed_ = left_circle_combo_pressed;

  // Square alone remains the manual end-effector velocity command.
  if (square_button_pressed && !option_button_pressed) {
    if (input_joy_cmd.axes.size() <= 3) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Joy message has fewer than four axes; velocity command was skipped.");
      return;
    }

    double vx = -input_joy_cmd.axes.at(0);
    double vy = input_joy_cmd.axes.at(1);
    double omega = input_joy_cmd.axes.at(3);

    std_msgs::msg::Float64MultiArray vel_msg;
    vel_msg.data.resize(3);

    vel_msg.data[0] = vx;
    vel_msg.data[1] = vy;
    vel_msg.data[2] = omega;

    ee_vel_pub_->publish(vel_msg);
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CommandInterface>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();

  return 0;
}
