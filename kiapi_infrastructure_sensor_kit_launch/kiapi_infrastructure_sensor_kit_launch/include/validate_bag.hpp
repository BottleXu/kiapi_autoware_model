#ifndef KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH__VALIDATE_BAG_HPP_
#define KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH__VALIDATE_BAG_HPP_

// ROS2 Components
#include "visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp>


// Nebula Components
// #include "autoware/point_types/types.hpp"
#include <boost/tokenizer.hpp>

#include <algorithm>
#include <ostream>
#include <string>
#include <memory>
#include <vector>
#include <chrono>
#include <functional>
#include <cmath>
#include <tuple>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


struct EIGEN_ALIGN16 PointXYZIRCAEDT
{
  PCL_ADD_POINT4D;
  float intensity;             // offset 16
  uint16_t ring;            // offset 20
  uint8_t return_type;         // offset 22
  float azimuth;               // offset 24
  float elevation;             // offset 28
  float distance;              // offset 32
  double time_stamp;           // offset 40
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PointXYZIRADT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  float azimuth;
  float distance;
  uint8_t return_type;
  double time_stamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRADT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(
    float, azimuth, azimuth)(float, distance, distance)(std::uint8_t, return_type, return_type)(
    double, time_stamp, time_stamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRCAEDT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (uint16_t, ring, ring)
  (uint8_t, return_type,return_type)
  (float, azimuth, azimuth)
  (float, elevation, elevation)
  (float, distance, distance)
  (double, time_stamp, time_stamp))


namespace kiapi_infrastructure
{

class PCLBagNode : public rclcpp::Node
{
public:
  KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_PUBLIC
  explicit PCLBagNode(const rclcpp::NodeOptions & options);
  sensor_msgs::msg::PointCloud2 synced_msg;
  sensor_msgs::msg::PointField pointfield_msg;
  int data_length;

  // const pcl::PointCloud<PointXYZIRCAEDT> input_pointcloud;

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  int64_t maximum_queue_size_ = 10;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  void convert_point_xyziradt(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /// @brief Converts degrees to radians
  /// @param radians
  /// @return degrees
  static inline float deg2rad(double degrees)
  {
    return degrees * M_PI / 180.0;
  }

  /// @brief Converts radians to degrees
  /// @param radians
  /// @return degrees
  static inline float rad2deg(double radians)
  {
    return radians * 180.0 / M_PI;
  }

}; // class

}  // namespace kiapi_infrastructure

#endif  // KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH__VALIDATE_BAG_HPP_
