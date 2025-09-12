#include "utm_lla.hpp"

UTMtoLLANode::UTMtoLLANode(const rclcpp::NodeOptions & options)
: Node("kiapi_gps_to_link", options)
{
  SetValues("North", 52, 6378137.0, 1.0/298.257223563, 0.9996);
  // WorldToMap();

  rclcpp::PublisherOptions pub_options;
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  subscription_gnss_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/novatel/oem7/fix", 10,
    std::bind(&UTMtoLLANode::pub_tf, this, std::placeholders::_1));

  subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/novatel/oem7/imu/data", 10,
    std::bind(&UTMtoLLANode::imu_callback, this, std::placeholders::_1));

  // Publisher: synced pointcloud
  publisher_path_ =  this->create_publisher<nav_msgs::msg::Path>(
    "path/lla_to_utm",
    rclcpp::SensorDataQoS().keep_last(maximum_queue_size_),
    pub_options);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UTMtoLLANode>());
  rclcpp::shutdown();
  return 0;
}
