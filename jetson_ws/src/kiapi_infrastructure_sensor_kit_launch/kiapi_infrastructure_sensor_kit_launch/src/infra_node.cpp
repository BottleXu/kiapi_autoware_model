#include <infra_node.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

namespace kiapi_infrastructure
{

PCLSyncNode::PCLSyncNode(const rclcpp::NodeOptions & options)
: Node("kiapi_hesai_sync", options)
{
  RCLCPP_INFO(this->get_logger(), "PointCloud Sync Node Initialized");

  rclcpp::PublisherOptions pub_options;
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

  // publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_raw_synced", 10);
  publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_raw_synced", rclcpp::SensorDataQoS().keep_last(maximum_queue_size_), pub_options);
  
  // auto callback = 
  // [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) -> void
  // {
  //   synced_msg = *msg;
  //   synced_msg.header.stamp = this->get_clock()->now();
  //   publisher_->publish(synced_msg);
  //   RCLCPP_INFO(this->get_logger(), "PointCloud Sync publish");
  // };
  // subscription_ = create_subscription<sensor_msgs::msg::PointCloud2("pointcloud_raw_ex", 10, callback);


  subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud_raw_ex", 
    rclcpp::SensorDataQoS().keep_last(maximum_queue_size_),
    std::bind(&PCLSyncNode::pointcloud_callback, this, std::placeholders::_1));
}

void PCLSyncNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  synced_msg = *msg;
  synced_msg.header.stamp = this->get_clock()->now();
  publisher_->publish(synced_msg);
  RCLCPP_INFO(this->get_logger(), "PointCloud Sync publish");
}

}  // namespace kiapi_infrastructure

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kiapi_infrastructure::PCLSyncNode)
