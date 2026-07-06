#include <algorithm>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

class EePathVisualizer : public rclcpp::Node
{
public:
  EePathVisualizer()
  : Node("pfr_ee_path_visualizer")
  {
    const auto desired_pose_topic = this->declare_parameter<std::string>(
      "desired_pose_topic", "/high_level/right/ee_pose_ref");
    const auto actual_pose_topic = this->declare_parameter<std::string>(
      "actual_pose_topic", "/current_ee_pose_R");
    const auto desired_path_topic = this->declare_parameter<std::string>(
      "desired_path_topic", "/pfr_visualization/right/desired_ee_path");
    const auto actual_path_topic = this->declare_parameter<std::string>(
      "actual_path_topic", "/pfr_visualization/right/actual_ee_path");
    default_frame_id_ = this->declare_parameter<std::string>("default_frame_id", "base_link");
    const auto max_path_points = this->declare_parameter<int64_t>("max_path_points", 3000);
    if (max_path_points <= 0) {
      throw std::invalid_argument("max_path_points must be positive");
    }
    max_path_points_ = static_cast<std::size_t>(max_path_points);

    const auto path_qos = rclcpp::QoS(1).reliable().transient_local();
    desired_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      desired_path_topic, path_qos);
    actual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      actual_path_topic, path_qos);

    desired_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      desired_pose_topic, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        appendPose(*msg, desired_path_, desired_path_pub_);
      });
    actual_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      actual_pose_topic, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        appendPose(*msg, actual_path_, actual_path_pub_);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "Visualizing desired '%s' and actual '%s' end-effector paths.",
      desired_pose_topic.c_str(), actual_pose_topic.c_str());
  }

private:
  void appendPose(
    const geometry_msgs::msg::PoseStamped & input_pose,
    nav_msgs::msg::Path & path,
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & publisher)
  {
    auto pose = input_pose;
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = default_frame_id_;
    }

    if (!path.header.frame_id.empty() && path.header.frame_id != pose.header.frame_id) {
      RCLCPP_WARN(
        this->get_logger(), "Path frame changed from '%s' to '%s'; clearing its history.",
        path.header.frame_id.c_str(), pose.header.frame_id.c_str());
      path.poses.clear();
    }

    path.header = pose.header;
    path.poses.push_back(std::move(pose));
    if (path.poses.size() > max_path_points_) {
      const auto excess = path.poses.size() - max_path_points_;
      path.poses.erase(path.poses.begin(), path.poses.begin() + static_cast<std::ptrdiff_t>(excess));
    }
    publisher->publish(path);
  }

  std::size_t max_path_points_{3000};
  std::string default_frame_id_{"base_link"};
  nav_msgs::msg::Path desired_path_;
  nav_msgs::msg::Path actual_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr desired_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr actual_pose_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EePathVisualizer>());
  rclcpp::shutdown();
  return 0;
}
