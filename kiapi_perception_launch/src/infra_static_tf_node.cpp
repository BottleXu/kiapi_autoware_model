#include "infra_static_tf_node.hpp"

InfraStaticTFNode::InfraStaticTFNode(const rclcpp::NodeOptions & options)
: Node("kiapi_infra_static_tf_node", options)
{
  SetValues("North", 52, 6378137.0, 1.0/298.257223563, 0.9996);
  rclcpp::PublisherOptions pub_options;
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  publisher_odom_ =  this->create_publisher<nav_msgs::msg::Odometry>(
    "infra/static_odom",
    rclcpp::SensorDataQoS().keep_last(maximum_queue_size_),
    pub_options);

  

  // yaml path
  yaml_path = declare_parameter<std::string>("yaml_path", "");
  is_map = declare_parameter<bool>("use_vector_map", false);
  ReadYaml();
  
  if (is_map){
      map_path = declare_parameter<std::string>("map_projector_info_path", "");
      RCLCPP_INFO(this->get_logger(), "map_projector_info_path: %s", map_path.c_str());

      ReadMapYaml();
      RCLCPP_INFO(this->get_logger(), "is_map::ReadMapYaml Done");
      MapInitSet();     
  }
  else{
    InitSet();
  }

  
  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate));
  
  if (is_map){
    timer_ = this->create_wall_timer(period, std::bind(&InfraStaticTFNode::map_timer_callback, this));
  }
  else{
    timer_ = this->create_wall_timer(period, std::bind(&InfraStaticTFNode::timer_callback, this));
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InfraStaticTFNode>());
  rclcpp::shutdown();
  return 0;
}



