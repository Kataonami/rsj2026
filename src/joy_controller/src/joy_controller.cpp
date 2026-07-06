#include "joy_controller/joy_controller.hpp"

JoyController::JoyController()
: Node("joy_controller")
{
  std::cout << "JoyController class is established." << std::endl;

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyController>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();

  return 0;
}