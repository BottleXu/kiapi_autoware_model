#include "gnss_to_link.hpp"

GnssLinkNode::GnssLinkNode(const rclcpp::NodeOptions & options)
: Node("kiapi_gps_to_link", options)
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  subscription_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/sensing/gnss/pose_with_covariance", 10,
    std::bind(&GnssLinkNode::pub_tf, this, std::placeholders::_1));

  subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/novatel/oem7/imu/data", 10,
    std::bind(&GnssLinkNode::imu_callback, this, std::placeholders::_1));
  

  rclcpp::PublisherOptions pub_options;
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  
  publisher_path_ =  this->create_publisher<nav_msgs::msg::Path>(
    "path/autoware_gnss",
    rclcpp::SensorDataQoS().keep_last(maximum_queue_size_),
    pub_options);
}

void GnssLinkNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  qx_ = msg->orientation.x;
  qy_ = msg->orientation.y;
  qz_ = msg->orientation.z;
  qw_ = msg->orientation.w;
}

void GnssLinkNode::pub_tf(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{   
  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header.stamp = msg->header.stamp;
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "autoware_gnss_link";

  tf_msg.transform.translation.x = msg->pose.pose.position.x;
  tf_msg.transform.translation.y = msg->pose.pose.position.y;
  tf_msg.transform.translation.z = msg->pose.pose.position.z;

  tf_msg.transform.rotation.x = qx_;
  tf_msg.transform.rotation.y = qy_;
  tf_msg.transform.rotation.z = qz_;
  tf_msg.transform.rotation.w = qw_;

  tf_broadcaster_->sendTransform(tf_msg);

  // Path Pub
  geometry_msgs::msg::PoseStamped poses;
  poses.header.stamp = msg->header.stamp;
  poses.header.frame_id = "map";

  poses.pose.position.x =  tf_msg.transform.translation.x;
  poses.pose.position.y =  tf_msg.transform.translation.y;
  poses.pose.position.z =  tf_msg.transform.translation.z;

  poses.pose.orientation = tf_msg.transform.rotation;

  path_.header.frame_id = "map";
  path_.header.stamp = msg->header.stamp;
  path_.poses.push_back(poses);

  publisher_path_->publish(path_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssLinkNode>());
  rclcpp::shutdown();
  return 0;
}
