#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

class KalmanFilterMockNode : public rclcpp::Node
{
public:
  KalmanFilterMockNode() : Node("kalman_filter_mock_node")
  {
    // Create publisher for package info
    package_info_pub_ = this->create_publisher<std_msgs::msg::String>(
      "kalman_filter/package_info", 10);
    
    // Create publisher for filtered odometry
    filtered_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "filtered_odometry", 10);
    
    // Create publisher for filtered pose
    filtered_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "filtered_pose", 10);
    
    // Create timer for periodic publishing
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&KalmanFilterMockNode::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Kalman Filter Mock Node started");
  }

private:
  void timer_callback()
  {
    // Publish package info
    auto package_msg = std_msgs::msg::String();
    package_msg.data = "kalman_filter: Mock Kalman filter for state estimation in Robium";
    package_info_pub_->publish(package_msg);
    
    // Publish mock filtered odometry
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "base_link";
    
    // Mock filtered position
    odom_msg.pose.pose.position.x = 1.0;
    odom_msg.pose.pose.position.y = 2.0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;
    
    // Mock filtered velocity
    odom_msg.twist.twist.linear.x = 0.5;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.1;
    
    // Mock covariance matrices
    for (int i = 0; i < 36; ++i) {
      odom_msg.pose.covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
      odom_msg.twist.covariance[i] = (i % 7 == 0) ? 0.1 : 0.0;
    }
    
    filtered_odom_pub_->publish(odom_msg);
    
    // Publish mock filtered pose
    auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "map";
    pose_msg.child_frame_id = "base_link";
    
    pose_msg.pose.pose.position.x = 1.0;
    pose_msg.pose.pose.position.y = 2.0;
    pose_msg.pose.pose.position.z = 0.0;
    pose_msg.pose.pose.orientation.x = 0.0;
    pose_msg.pose.pose.orientation.y = 0.0;
    pose_msg.pose.pose.orientation.z = 0.0;
    pose_msg.pose.pose.orientation.w = 1.0;
    
    // Mock pose covariance
    for (int i = 0; i < 36; ++i) {
      pose_msg.pose.covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
    }
    
    filtered_pose_pub_->publish(pose_msg);
  }
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr package_info_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr filtered_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KalmanFilterMockNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 