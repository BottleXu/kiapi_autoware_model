#ifndef KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH__INFRA_NODE_HPP_
#define KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH__INFRA_NODE_HPP_

#include "visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace kiapi_infrastructure
{

class PCLSyncNode : public rclcpp::Node
{
public:
  KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_PUBLIC
  explicit PCLSyncNode(const rclcpp::NodeOptions & options);
  sensor_msgs::msg::PointCloud2 synced_msg;

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  int64_t maximum_queue_size_ = 10;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace kiapi_infrastructure

#endif  // KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH__INFRA_NODE_HPP_
