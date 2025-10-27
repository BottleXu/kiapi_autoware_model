#include "pointcloud_xyzirt_bypass_node.hpp"

using namespace std::chrono_literals;


PCLConvertNode::PCLConvertNode(const rclcpp::NodeOptions & options)
: Node("kiapi_bypass", options)
{
  RCLCPP_INFO(this->get_logger(), "PointCloud Sync Node Initialized");

  rclcpp::PublisherOptions pub_options;
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

  // Publisher: synced pointcloud
  publisher_ =  this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/sensing/lidar/pointcloud",
    rclcpp::SensorDataQoS().keep_last(maximum_queue_size_),
    pub_options);

  // Subscribers
  subscription_ =  this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/hesai/pandar", 
    rclcpp::SensorDataQoS().keep_last(maximum_queue_size_),
    std::bind(&PCLConvertNode::convert_point_xyzirc, this, std::placeholders::_1));
  
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCLConvertNode>());
  rclcpp::shutdown();
  return 0;
}
