#ifndef SIM_PARAMETER_HPP  
#define SIM_PARAMETER_HPP

#include <string>
#include <array>
#include <Eigen/Dense>



// body name
const std::string target_body_name = "target";
const std::string chaser_body_name = "chaser";

// joint_state name 
std::vector<std::string> target_base_joint_names = {"target_x", "target_y", "target_yaw"};
std::vector<std::string> chaser_base_joint_names = {"chaser_x", "chaser_y", "chaser_yaw"};
std::vector<std::string> chaser_arm_joint_names = {"joint1", "joint2", "joint3"};

// joint state
std::vector<double> target_base_joint_positions = {0.0, 0.0, 0.0}; 
std::vector<double> chaser_base_joint_positions = {0.0, 0.0, 0.0};
std::vector<double> chaser_arm_joint_positions = {0.0, 0.0, 0.0};

std::vector<double> target_base_joint_velocities = {0.0, 0.0, 0.0}; 
std::vector<double> chaser_base_joint_velocities = {0.0, 0.0, 0.0};
std::vector<double> chaser_arm_joint_velocities = {0.0, 0.0, 0.0};

// sensor name
std::vector<std::string> chaser_base_pos = {"chaser_x_pos", "chaser_y_pos", "chaser_yaw_pos"};
std::vector<std::string> chaser_base_vel = {"chaser_x_vel", "chaser_y_vel", "chaser_yaw_vel"};

std::vector<std::string> chaser_arm_pos = {"joint1_pos", "joint2_pos", "joint3_pos"};
std::vector<std::string> chaser_arm_vel = {"joint1_vel", "joint2_vel", "joint3_vel"};

std::vector<std::string> force_sensor_names = {"ee_force"};
std::vector<std::string> torque_sensor_names = {"ee_torque"};

std::vector<std::string> ee_pos_sensor_names = {"ee_pos"};
std::vector<std::string> ee_linvel_names = {"ee_linvel"};

#endif // SIM_PARAMETER_HPP