#ifndef JOY_CONTROLLER_HPP
#define JOY_CONTROLLER_HPP

#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoyController : public rclcpp::Node
{
public:
  JoyController();
private:
  
};

#endif // JOY_CONTROLLER_HPP