#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <sensor_msgs/msg/image.hpp>

class PersonTrackingMockNode : public rclcpp::Node
{
public:
  PersonTrackingMockNode() : Node("person_tracking_mock_node")
  {
    // Create publisher for package info
    package_info_pub_ = this->create_publisher<std_msgs::msg::String>(
      "person_tracking/package_info", 10);
    
    // Create publisher for person detections
    person_detections_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(
      "person_detections", 10);
    
    // Create publisher for person poses
    person_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "person_poses", 10);
    
    // Create timer for periodic publishing
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&PersonTrackingMockNode::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Person Tracking Mock Node started");
  }

private:
  void timer_callback()
  {
    // Publish package info
    auto package_msg = std_msgs::msg::String();
    package_msg.data = "person_tracking: Mock person tracking and detection package for Robium";
    package_info_pub_->publish(package_msg);
    
    // Publish mock person detections
    auto detections_msg = vision_msgs::msg::Detection3DArray();
    detections_msg.header.stamp = this->now();
    detections_msg.header.frame_id = "camera_frame";
    
    // Mock detection
    vision_msgs::msg::Detection3D detection;
    detection.header.stamp = this->now();
    detection.header.frame_id = "camera_frame";
    detection.bbox.center.position.x = 2.0;
    detection.bbox.center.position.y = 1.0;
    detection.bbox.center.position.z = 0.0;
    detection.bbox.size.x = 0.5;
    detection.bbox.size.y = 0.3;
    detection.bbox.size.z = 1.8;
    detection.results.resize(1);
    detection.results[0].hypothesis.class_id = "person";
    detection.results[0].hypothesis.score = 0.95;
    
    detections_msg.detections.push_back(detection);
    person_detections_pub_->publish(detections_msg);
    
    // Publish mock person poses
    auto poses_msg = geometry_msgs::msg::PoseArray();
    poses_msg.header.stamp = this->now();
    poses_msg.header.frame_id = "map";
    
    geometry_msgs::msg::Pose person_pose;
    person_pose.position.x = 2.0;
    person_pose.position.y = 1.0;
    person_pose.position.z = 0.0;
    person_pose.orientation.x = 0.0;
    person_pose.orientation.y = 0.0;
    person_pose.orientation.z = 0.0;
    person_pose.orientation.w = 1.0;
    
    poses_msg.poses.push_back(person_pose);
    person_poses_pub_->publish(poses_msg);
  }
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr package_info_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr person_detections_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr person_poses_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PersonTrackingMockNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 