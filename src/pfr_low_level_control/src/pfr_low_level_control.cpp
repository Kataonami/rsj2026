#include <memory>

#include "pfr_low_level_control/pfr_low_level_control.hpp"

PfrLowLevelControl::PfrLowLevelControl()
: Node("pfr_low_level_control")
{
  RCLCPP_INFO(this->get_logger(), "PFR low-level controller started.");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PfrLowLevelControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
