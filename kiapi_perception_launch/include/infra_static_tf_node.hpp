// ROS2 Components
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <autoware_map_msgs/msg/map_projector_info.hpp>

#include <cmath>
#include <vector>
#include <memory>
#include <iostream>

#include <yaml-cpp/yaml.h>
#include <GeographicLib/TransverseMercator.hpp>

//resource from wiki  Universal Transverse Mercator coordinate system. WGS 84
enum Hemi {NorthH, SouthH};

struct lla_data{
  double lat = 0.0;//-594929.9431329881;//east
  double lon = 0.0;//-4139043.529676078;//north
  double alt = 0.0;//unit in m
};// map origin point (LLA)

struct utm_data{
  double bx = 445724.933;//-594929.9431329881;//east
  double by = 3945031.452;//-4139043.529676078;//north
  double bz = 0.0;//unit in m
};// map origin point (UTM)

class InfraStaticTFNode: public rclcpp::Node
{
  public:
  explicit InfraStaticTFNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  void SetValues(std::string hemi, int zone, double at, double fla, double k0); // :tm_(at,fla,k0),zone_(zone), hemi_(hemi);
  std::vector<double> get_lla();



  private:

  const double kNN_      = 0;
  const double kNS_      = 10000000.0;
  const double kE0_      = 500000.0;
  const double kPI_      = 3.14159265359;
  const double kDist_    = 1.2;

  std::unique_ptr<GeographicLib::TransverseMercator> tm_;
  std::vector<double> lla_{3,0.0}, utm_{3,0.0}, world_utm_{3,0,0};

  // Vector map info Sub
  rclcpp::Subscription<autoware_map_msgs::msg::MapProjectorInfo>::SharedPtr subscription_mapinfo_;
  

  // TF broadcaster, Publisher
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom_;

  // func
  void ReadYaml();
  void ReadMapYaml();
  void InitSet();
  utm_data LLAConvert2UTM(Hemi hemi, int zone, double latitude, double longitude, double altitude);
  void MapinfoCallback(const autoware_map_msgs::msg::MapProjectorInfo::SharedPtr msg);
  void timer_callback();
  

  // Callbacks
  rclcpp::TimerBase::SharedPtr timer_;
  
  
  std::string yaml_path;
  std::string map_path;
  double heading_yaw;
  size_t tmp_counter = 0;
  double tmp_x, tmp_y, tmp_z;
  std::vector<double> his_x,his_y,his_z;
  int zone_;
  std::string hemi_;

  int64_t maximum_queue_size_ = 10;

  // Conversion storage

  tf2::Quaternion tf2_quat;
  geometry_msgs::msg::TransformStamped tf_msg;
  nav_msgs::msg::Odometry odom_msg;
  int rate;
  
  lla_data lla_origin_data;
  lla_data lla_map_data;
  utm_data utm_origin_data;
  utm_data utm_infra_data;

  
  bool is_set = false;
  bool is_map;

};

void InfraStaticTFNode::SetValues(std::string hemi, int zone, double at, double fla, double k0)
{
    hemi_ = hemi;
    zone_ = zone;
    tm_ = std::make_unique<GeographicLib::TransverseMercator>(at, fla, k0);
}


utm_data InfraStaticTFNode::LLAConvert2UTM(Hemi hemi, int zone, double latitude, double longitude, double height){
  double east,north;
  utm_data utm_;

  int lon0  = zone*6-183;

  tm_->Forward(lon0, latitude, longitude, east, north);

  east += kE0_;
  north += (hemi==NorthH) ? kNN_ : kNS_;

  utm_.bx = east;
  utm_.by = north;
  utm_.bz = height;
  return utm_;
}


void InfraStaticTFNode::ReadYaml()
{
  if (yaml_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "YAML path is empty");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "YAML path: %s", yaml_path.c_str());

  YAML::Node config;
  try {
    config = YAML::LoadFile(yaml_path);
  } catch (const YAML::BadFile &e) {
    RCLCPP_FATAL(this->get_logger(), "Cannot open YAML file: %s", e.what());
    return;
  }

  // Access ROS 2-style parameters
  YAML::Node params;
  if (config["/**"] && config["/**"]["ros__parameters"]) {
    params = config["/**"]["ros__parameters"];
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid YAML structure. Missing /**/ros__parameters block");
    return;
  }

  // Read values safely with defaults
  try {
    rate = params["rate"] ? params["rate"].as<int>() : 10;

    if (params["gnss"]) {
      lla_origin_data.lat = params["gnss"]["latitude"].as<double>();
      lla_origin_data.lon = params["gnss"]["longitude"].as<double>();
      lla_origin_data.alt = params["gnss"]["altitude"].as<double>();
    } else {
      RCLCPP_WARN(this->get_logger(), "Missing 'gnss' block in YAML");
    }

    if (params["imu"] && params["imu"]["heading_yaw"]) {
      heading_yaw = params["imu"]["heading_yaw"].as<double>();
    } else {
      heading_yaw = 0.0;
      RCLCPP_WARN(this->get_logger(), "Missing 'imu.heading_yaw' parameter, defaulting to 0.0");
    }

    tf2_quat.setRPY(0.0, 0.0, heading_yaw);

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s", e.what());
  }
}

void InfraStaticTFNode::ReadMapYaml()
{
  if (map_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "map path is empty");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "map_path: %s", map_path.c_str());

  YAML::Node config;
  try {
    config = YAML::LoadFile(map_path);
  } catch (const YAML::BadFile &e) {
    RCLCPP_FATAL(this->get_logger(), "Cannot open YAML file: %s", e.what());
    return;
  }
  lla_map_data.lat = config["map_origin"]["latitude"].as<double>();
  lla_map_data.lat = config["map_origin"]["longitude"].as<double>();
  lla_map_data.lat = config["map_origin"]["altitude"].as<double>();
  
}

void InfraStaticTFNode::MapinfoCallback(const autoware_map_msgs::msg::MapProjectorInfo::SharedPtr msg){
  lla_map_data.lat = msg->map_origin.latitude;
  lla_map_data.lon = msg->map_origin.longitude;
  lla_map_data.alt = msg->map_origin.altitude;
  std::cout << lla_map_data.lat << std::endl;
}

void InfraStaticTFNode::InitSet(){ 
  // lla to utm
  if (!tm_) {
  RCLCPP_ERROR(this->get_logger(), "TransverseMercator not initialized. Call SetValues first.");
  return;}

  // set transform msg
  tf_msg.header.stamp = this->get_clock()->now();
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "base_link";
  utm_infra_data = LLAConvert2UTM(NorthH, zone_, lla_origin_data.lat, lla_origin_data.lon, lla_origin_data.alt);

  if (is_map){
    utm_origin_data = LLAConvert2UTM(NorthH, zone_, lla_map_data.lat, lla_map_data.lon, lla_map_data.alt);
  
    tf_msg.transform.translation.x = utm_origin_data.bx - utm_infra_data.bx;
    tf_msg.transform.translation.y = utm_origin_data.by - utm_infra_data.by;
    tf_msg.transform.translation.z = utm_origin_data.bz - utm_infra_data.bz;
  }
  else{
    tf_msg.transform.translation.x = utm_infra_data.bx;
    tf_msg.transform.translation.y = utm_infra_data.by;
    tf_msg.transform.translation.z = utm_infra_data.bz; 
  }

  tf_msg.transform.rotation.x = tf2_quat[0];
  tf_msg.transform.rotation.y = tf2_quat[1];
  tf_msg.transform.rotation.z = tf2_quat[2];
  tf_msg.transform.rotation.w = tf2_quat[3];
  
  
  

  

  tf_broadcaster_->sendTransform(tf_msg);

  odom_msg.header.stamp = tf_msg.header.stamp;
  odom_msg.header.frame_id = tf_msg.header.frame_id;
  odom_msg.child_frame_id = tf_msg.child_frame_id;

  odom_msg.pose.pose.position.x = tf_msg.transform.translation.x;
  odom_msg.pose.pose.position.y = tf_msg.transform.translation.y;
  odom_msg.pose.pose.position.z = tf_msg.transform.translation.z;

  odom_msg.pose.pose.orientation.x = tf_msg.transform.rotation.x;
  odom_msg.pose.pose.orientation.y = tf_msg.transform.rotation.y;
  odom_msg.pose.pose.orientation.z = tf_msg.transform.rotation.z;
  odom_msg.pose.pose.orientation.w = tf_msg.transform.rotation.w;

  publisher_odom_->publish(odom_msg);

  is_set = true;
}

void InfraStaticTFNode::timer_callback(){
  tf_msg.header.stamp = this->get_clock()->now();
  tf_broadcaster_->sendTransform(tf_msg);

  odom_msg.header.stamp = tf_msg.header.stamp;
  publisher_odom_->publish(odom_msg);
}