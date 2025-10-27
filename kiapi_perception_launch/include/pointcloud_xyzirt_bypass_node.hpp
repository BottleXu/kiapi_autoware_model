// ROS2 Components
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


struct EIGEN_ALIGN16 PointXYZIRC
{
  PCL_ADD_POINT4D;
  uint8_t intensity;             
  uint8_t return_type;         
  uint8_t channel;         
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRC,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (std::uint8_t, intensity, intensity)
  (std::uint8_t, return_type, return_type)
  (std::uint16_t, channel, channel))




class PCLConvertNode : public rclcpp::Node
{
public:
  explicit PCLConvertNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  sensor_msgs::msg::PointCloud2 synced_msg;
  sensor_msgs::msg::PointField pointfield_msg;
  int data_length;

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  void convert_point_xyzirc(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

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
  // queue size parameter
  int64_t maximum_queue_size_ = 10;
};



void PCLConvertNode::convert_point_xyzirc(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{ 

  const size_t point_count = msg->width * msg->height;

  sensor_msgs::msg::PointCloud2 output;
  output.header = msg->header;
  output.height = 1;
  output.width = point_count;
  output.is_bigendian = false;
  output.is_dense = true;
  output.point_step = 16; 
  output.row_step = output.point_step * output.width;
  output.data.resize(output.row_step);

  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.offset = 0;
  field.datatype = 7;
  field.count = 1;
  output.fields.push_back(field);

  field.name = "y";
  field.offset = 4;
  field.datatype = 7;
  field.count = 1;
  output.fields.push_back(field);

  field.name = "z";
  field.offset = 8;
  field.datatype = 7;
  field.count = 1;
  output.fields.push_back(field);

  field.name = "intensity";
  field.offset = 12;
  field.datatype = 2;
  field.count = 1;
  output.fields.push_back(field);

  field.name = "return_type";
  field.offset = 13;
  field.datatype = 2;
  field.count = 1;
  output.fields.push_back(field);

  field.name = "channel";
  field.offset = 14;
  field.datatype = 4;
  field.count = 1;
  output.fields.push_back(field);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity(*msg, "intensity");
  sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(*msg, "ring");

  
  for (size_t i = 0; i < point_count;
      ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_ring)
  {
    uint8_t * ptr = &output.data[i * output.point_step];

    *reinterpret_cast<float*>(ptr + 0) = *iter_x;
    *reinterpret_cast<float*>(ptr + 4) = *iter_y;
    *reinterpret_cast<float*>(ptr + 8) = *iter_z;

    // intensity: float32 -> uint8

    float raw_intensity = *iter_intensity;  // always float32 in your input
    raw_intensity = std::clamp(raw_intensity, 0.0f, 255.0f);
    ptr[12] = static_cast<uint8_t>(raw_intensity);
    ptr[13] = 0; // return_type (default or sensor-dependent)

    // ring -> channel
    *reinterpret_cast<uint16_t*>(ptr + 14) = *iter_ring;
  }

  output.header.frame_id = "base_link";
  output.header.stamp = this->get_clock()->now();

  publisher_->publish(output);
}
  

