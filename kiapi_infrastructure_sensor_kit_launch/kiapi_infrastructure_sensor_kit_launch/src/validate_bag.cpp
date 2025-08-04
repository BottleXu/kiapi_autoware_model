#include <validate_bag.hpp>

// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>

// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <sensor_msgs/point_cloud2_iterator.hpp>
// #include <sensor_msgs/msg/point_field.hpp>

using namespace std::chrono_literals;

namespace kiapi_infrastructure
{

PCLBagNode::PCLBagNode(const rclcpp::NodeOptions & options)
: Node("kiapi_hesai_nebulr_bag", options)
{
  RCLCPP_INFO(this->get_logger(), "PointCloud From nebulr_bag Node Initialized");

  rclcpp::PublisherOptions pub_options;
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

  publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_raw_synced", rclcpp::SensorDataQoS().keep_last(maximum_queue_size_), pub_options);
  
  subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/hesai/pandar", 
    rclcpp::SensorDataQoS().keep_last(maximum_queue_size_),
    std::bind(&PCLBagNode::convert_point_xyziradt, this, std::placeholders::_1));
}

void PCLBagNode::convert_point_xyziradt(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{ 
  // pointcloud2 data format converter
  // from
  // sub_msg = synced_msg =(6) [x:0-3][y:4-7][z:8-12][Intensity:16-23][Timestamp:24-31][ring:32-]
  // to
  // pub_msg = [x:0-3][y:4-7][z:8-12][Intensity:16-19][Ring:20-23][Azimuth:24-27][Distance:28-31][Return_type:32-39][Time_stamp:40-]

  const size_t point_count = msg->width * msg->height;

  sensor_msgs::msg::PointCloud2 output;
  output.header = msg->header;
  output.height = 1;
  output.width = point_count;
  output.is_bigendian = false;
  output.is_dense = true;
  output.point_step = 36;  // NebulaPointXYZIRTCAE layout
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

  field.name = "azimuth";
  field.offset = 16;
  field.datatype = 7;
  field.count = 1;
  output.fields.push_back(field);

  field.name = "elevation";
  field.offset = 20;
  field.datatype = 7;
  field.count = 1;
  output.fields.push_back(field);

  field.name = "distance";
  field.offset = 24;
  field.datatype = 7;
  field.count = 1;
  output.fields.push_back(field);

  field.name = "time_stamp";
  field.offset = 28;
  field.datatype = 6;
  field.count = 1;
  output.fields.push_back(field);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
  sensor_msgs::PointCloud2ConstIterator<double> iter_timestamp(*msg, "timestamp");
  sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_ring(*msg, "ring");

  for (size_t i = 0; i < point_count;
        ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_timestamp, ++iter_ring)
  {
    uint8_t * ptr = &output.data[i * output.point_step];

    *reinterpret_cast<float*>(ptr + 0) = *iter_x;
    *reinterpret_cast<float*>(ptr + 4) = *iter_y;
    *reinterpret_cast<float*>(ptr + 8) = *iter_z;

    ptr[12] = static_cast<uint8_t>(std::clamp(*iter_intensity, 0.0f, 255.0f)); // quantized
    ptr[13] = 0; // return_type (default or sensor-dependent)
    *reinterpret_cast<uint32_t*>(ptr + 14) = *iter_ring; // channel

    *reinterpret_cast<float*>(ptr + 16) = 0.0f; // azimuth (optional)
    *reinterpret_cast<float*>(ptr + 20) = 0.0f; // elevation (optional)
    *reinterpret_cast<float*>(ptr + 24) = std::sqrt(
      (*iter_x)*(*iter_x) + (*iter_y)*(*iter_y) + (*iter_z)*(*iter_z)); // distance

    *reinterpret_cast<uint64_t*>(ptr + 28) = static_cast<uint64_t>(*iter_timestamp);
  }
  output.header.frame_id = "hesai_lidar";
  output.header.stamp = this->get_clock()->now();

  publisher_->publish(output);
}
  


}  // namespace kiapi_infrastructure

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kiapi_infrastructure::PCLBagNode)
