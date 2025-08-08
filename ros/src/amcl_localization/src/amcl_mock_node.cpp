#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class AmclMockNode : public rclcpp::Node
{
public:
  AmclMockNode() : Node("amcl_mock_node")
  {
    // Create publisher for package info
    package_info_pub_ = this->create_publisher<std_msgs::msg::String>(
      "amcl_localization/package_info", 10);
    
    // Create publisher for pose estimate (mock AMCL output)
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose", 10);
    
    // Create transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    
    // Create timer for periodic publishing
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&AmclMockNode::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "AMCL Localization Mock Node started");
  }

private:
  void timer_callback()
  {
    // Publish package info
    auto package_msg = std_msgs::msg::String();
    package_msg.data = "amcl_localization: Mock AMCL (Adaptive Monte Carlo Localization) package for Robium";
    package_info_pub_->publish(package_msg);
    
    // Publish mock pose estimate
    auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "map";
    pose_msg.child_frame_id = "base_link";
    
    // Mock pose (origin)
    pose_msg.pose.pose.position.x = 0.0;
    pose_msg.pose.pose.position.y = 0.0;
    pose_msg.pose.pose.position.z = 0.0;
    
    pose_msg.pose.pose.orientation.x = 0.0;
    pose_msg.pose.pose.orientation.y = 0.0;
    pose_msg.pose.pose.orientation.z = 0.0;
    pose_msg.pose.pose.orientation.w = 1.0;
    
    // Mock covariance (identity matrix)
    for (int i = 0; i < 36; ++i) {
      pose_msg.pose.covariance[i] = (i % 7 == 0) ? 0.1 : 0.0;
    }
    
    pose_pub_->publish(pose_msg);
    
    // Publish transform
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    
    tf_broadcaster_->sendTransform(transform);
  }
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr package_info_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AmclMockNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 