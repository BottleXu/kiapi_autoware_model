#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <memory>

class GnssLinkNode : public rclcpp::Node
{
public:
  explicit GnssLinkNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_pose_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;

  // TF broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_path_;


  // Quaternion storage
  double qx_ = 0.0, qy_ = 0.0, qz_ = 0.0, qw_ = 1.0;
  // Path storage
  nav_msgs::msg::Path path_;
  int64_t maximum_queue_size_ = 10;

  // Callbacks
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void pub_tf(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
};
