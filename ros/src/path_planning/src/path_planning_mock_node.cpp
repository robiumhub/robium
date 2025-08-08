#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

class PathPlanningMockNode : public rclcpp::Node
{
public:
  PathPlanningMockNode() : Node("path_planning_mock_node")
  {
    // Create publisher for package info
    package_info_pub_ = this->create_publisher<std_msgs::msg::String>(
      "path_planning/package_info", 10);
    
    // Create publisher for planned path
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "planned_path", 10);
    
    // Create publisher for velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 10);
    
    // Create timer for periodic publishing
    timer_ = this->create_wall_timer(
      std::chrono::seconds(3),
      std::bind(&PathPlanningMockNode::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Path Planning Mock Node started");
  }

private:
  void timer_callback()
  {
    // Publish package info
    auto package_msg = std_msgs::msg::String();
    package_msg.data = "path_planning: Mock path planning and navigation package for Robium";
    package_info_pub_->publish(package_msg);
    
    // Publish mock planned path
    auto path_msg = nav_msgs::msg::Path();
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";
    
    // Create a simple path (straight line)
    for (int i = 0; i < 10; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = this->now();
      pose.header.frame_id = "map";
      pose.pose.position.x = i * 0.5;
      pose.pose.position.y = 0.0;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      
      path_msg.poses.push_back(pose);
    }
    
    path_pub_->publish(path_msg);
    
    // Publish mock velocity command
    auto cmd_vel_msg = geometry_msgs::msg::Twist();
    cmd_vel_msg.linear.x = 0.5;  // Move forward at 0.5 m/s
    cmd_vel_msg.angular.z = 0.0; // No rotation
    
    cmd_vel_pub_->publish(cmd_vel_msg);
  }
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr package_info_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPlanningMockNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 