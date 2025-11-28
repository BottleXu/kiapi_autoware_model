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
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
};

struct utm_data{
  double px = 0.0;
  double py = 0.0;
  double pz = 0.0; 
};

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
    void MapInitSet();
    utm_data LLAConvert2UTM(Hemi hemi, int zone, double latitude, double longitude, double altitude);
    void MapinfoCallback(const autoware_map_msgs::msg::MapProjectorInfo::SharedPtr msg);
    void timer_callback();
    void map_timer_callback();
    

    // Callbacks
    rclcpp::TimerBase::SharedPtr timer_;
    
    
    std::string yaml_path;
    std::string map_path;
    double px;
    double py;
    double pz;
    double heading_yaw;
    size_t tmp_counter = 0;
    double tmp_x, tmp_y, tmp_z;
    std::vector<double> his_x,his_y,his_z;
    int zone_;
    std::string hemi_;

    int64_t maximum_queue_size_ = 10;

    // Conversion storage

    tf2::Quaternion lidar_quat;
    geometry_msgs::msg::TransformStamped world_tf;
    geometry_msgs::msg::TransformStamped pole_tf;
    geometry_msgs::msg::TransformStamped lidar_tf;
    nav_msgs::msg::Odometry odom_msg;
    int rate;
    
    lla_data map_origin_lla;
    lla_data callback_origin_lla;
    lla_data pole_origin_lla;

    utm_data map_origin_utm;
    utm_data pole_origin_utm;
    
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

  utm_.px = east;
  utm_.py = north;
  utm_.pz = height;
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
    if (params["pole"]) {
      pole_origin_lla.lat = params["pole"]["latitude"].as<double>();
      pole_origin_lla.lon = params["pole"]["longitude"].as<double>();
      pole_origin_lla.alt = params["pole"]["altitude"].as<double>();
    } else {
      RCLCPP_WARN(this->get_logger(), "Missing 'pole' block in YAML");
    }

    if (params["lidar"]) {
      px = params["lidar"]["px"].as<double>();
      py = params["lidar"]["py"].as<double>();
      pz = params["lidar"]["pz"].as<double>();

      heading_yaw = params["lidar"]["heading_yaw"].as<double>();
    } else {
      heading_yaw = 0.0;
      RCLCPP_WARN(this->get_logger(), "Missing 'imu.heading_yaw' parameter, defaulting to 0.0");
    }
    double deg2rad = M_PI / 180.0;
    lidar_quat.setRPY(0.0, 0.0, heading_yaw * deg2rad);

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "ReadYaml::YAML parsing error: %s", e.what());
  }
}

void InfraStaticTFNode::ReadMapYaml()
{
  if (map_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "map path is empty");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "MAP_path: %s", map_path.c_str());

  YAML::Node config;

  try {
    config = YAML::LoadFile(map_path);
  } 
  
  catch (const YAML::BadFile &e) {
    RCLCPP_FATAL(this->get_logger(), "Cannot open MAP YAML file: %s", e.what());
    return;
  }
  
  catch (const std::exception &e) {
    RCLCPP_FATAL(this->get_logger(), "Error loading MAP YAML file: %s", e.what());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Open map: %s", map_path.c_str());

  // std::cout <<"config: "<< config["map_origin"]["latitude"] << std::endl;
  map_origin_lla.lat = config["map_origin"]["latitude"].as<double>();
  // std::cout <<"latitude: "<< map_origin_lla.lat << std::endl;


  // std::cout <<"config: "<< config["map_origin"]["longitude"] << std::endl;
  map_origin_lla.lon = config["map_origin"]["longitude"].as<double>();
  // std::cout <<"longitude: "<< map_origin_lla.lon << std::endl;


  // std::cout <<"config: "<< config["map_origin"]["altitude"] << std::endl;
  map_origin_lla.alt = config["map_origin"]["altitude"].as<double>();
  // std::cout <<"altitude: "<< map_origin_lla.alt << std::endl;

}

void InfraStaticTFNode::MapinfoCallback(const autoware_map_msgs::msg::MapProjectorInfo::SharedPtr msg){
  callback_origin_lla.lat = msg->map_origin.latitude;
  callback_origin_lla.lon = msg->map_origin.longitude;
  callback_origin_lla.alt = msg->map_origin.altitude;
  // std::cout << callback_origin_lla.lat << std::endl;
}

void InfraStaticTFNode::InitSet(){
  // 1. get pole UTM
  if (!tm_) {
    RCLCPP_ERROR(this->get_logger(), "TransverseMercator not initialized. Call SetValues first.");
    return;
  }
  pole_origin_utm = LLAConvert2UTM(NorthH, zone_, pole_origin_lla.lat, pole_origin_lla.lon, pole_origin_lla.alt);

  // 2. Set TF: map->infar_pole; UTM
  pole_tf.header.stamp = this->get_clock()->now();
  pole_tf.header.frame_id = "map";
  pole_tf.child_frame_id = "infra_pole";

  pole_tf.transform.translation.x = pole_origin_utm.px;
  pole_tf.transform.translation.y = pole_origin_utm.py;
  pole_tf.transform.translation.z = pole_origin_utm.pz; 

  pole_tf.transform.rotation.x = 0;
  pole_tf.transform.rotation.y = 0;
  pole_tf.transform.rotation.z = 0;
  pole_tf.transform.rotation.w = 1;
  tf_broadcaster_->sendTransform(pole_tf);


  // 3. Set TF: infra_pole->lidar; local
  lidar_tf.header.stamp = this->get_clock()->now();
  lidar_tf.header.frame_id = "infra_pole";
  lidar_tf.child_frame_id = "base_link";

  lidar_tf.transform.translation.x = px;
  lidar_tf.transform.translation.y = py;
  lidar_tf.transform.translation.z = pz; 

  lidar_tf.transform.rotation.x = lidar_quat[0];
  lidar_tf.transform.rotation.y = lidar_quat[1];
  lidar_tf.transform.rotation.z = lidar_quat[2];
  lidar_tf.transform.rotation.w = lidar_quat[3];
  tf_broadcaster_->sendTransform(lidar_tf);


  odom_msg.header.stamp = lidar_tf.header.stamp;
  odom_msg.header.frame_id ="map";
  odom_msg.child_frame_id = "base_link";

  odom_msg.pose.pose.position.x = lidar_tf.transform.translation.x;
  odom_msg.pose.pose.position.y = lidar_tf.transform.translation.y;
  odom_msg.pose.pose.position.z = lidar_tf.transform.translation.z;

  odom_msg.pose.pose.orientation.x = lidar_tf.transform.rotation.x;
  odom_msg.pose.pose.orientation.y = lidar_tf.transform.rotation.y;
  odom_msg.pose.pose.orientation.z = lidar_tf.transform.rotation.z;
  odom_msg.pose.pose.orientation.w = lidar_tf.transform.rotation.w;

  publisher_odom_->publish(odom_msg);

  is_set = true;
}

void InfraStaticTFNode::MapInitSet(){
  // 1. get map, pole UTM
  if (!tm_) {
    RCLCPP_ERROR(this->get_logger(), "TransverseMercator not initialized. Call SetValues first.");
    return;
  }

  map_origin_utm = LLAConvert2UTM(NorthH, zone_, map_origin_lla.lat, map_origin_lla.lon, map_origin_lla.alt);
  pole_origin_utm = LLAConvert2UTM(NorthH, zone_, pole_origin_lla.lat, pole_origin_lla.lon, pole_origin_lla.alt);

  // 2. Set TF: world -> map; UTM
  world_tf.header.stamp = this->get_clock()->now();
  world_tf.header.frame_id = "world";
  world_tf.child_frame_id = "map";

  world_tf.transform.translation.x = map_origin_utm.px;
  world_tf.transform.translation.y = map_origin_utm.py;
  world_tf.transform.translation.z = map_origin_utm.pz; 

  world_tf.transform.rotation.x = 0;
  world_tf.transform.rotation.y = 0;
  world_tf.transform.rotation.z = 0;
  world_tf.transform.rotation.w = 1;
  tf_broadcaster_->sendTransform(world_tf);

  // 3. Set TF: map -> pole; local
  pole_tf.header.stamp = this->get_clock()->now();
  pole_tf.header.frame_id = "map";
  pole_tf.child_frame_id = "infra_pole";

  // std::cout <<"map_origin_utm x: "<< map_origin_utm.px << std::endl;
  // std::cout <<"pole_origin_utm x: "<< pole_origin_utm.px << std::endl;
  // std::cout <<"diff x: "<< (pole_origin_utm.px - map_origin_utm.px) << std::endl;


  pole_tf.transform.translation.x = pole_origin_utm.px - map_origin_utm.px;
  pole_tf.transform.translation.y = pole_origin_utm.py - map_origin_utm.py;
  pole_tf.transform.translation.z = pole_origin_utm.pz - map_origin_utm.pz; 

  if (pole_tf.transform.translation.z < 0){
    pole_tf.transform.translation.z *= -1;
  }

  // RCLCPP_INFO(this->get_logger(), "pole_tf: x: %f", pole_tf.transform.translation.x);
  // RCLCPP_INFO(this->get_logger(), "pole_tf: y: %f", pole_tf.transform.translation.y);
  // RCLCPP_INFO(this->get_logger(), "pole_tf: z: %f", pole_tf.transform.translation.z);

  pole_tf.transform.rotation.x = 0;
  pole_tf.transform.rotation.y = 0;
  pole_tf.transform.rotation.z = 0;
  pole_tf.transform.rotation.w = 1;
  tf_broadcaster_->sendTransform(pole_tf);

  


  // 4. Set TF: infra_pole->lidar; local
  lidar_tf.header.stamp = this->get_clock()->now();
  lidar_tf.header.frame_id = "infra_pole";
  lidar_tf.child_frame_id = "base_link";

  lidar_tf.transform.translation.x = px;
  lidar_tf.transform.translation.y = py;
  lidar_tf.transform.translation.z = pz; 

  lidar_tf.transform.rotation.x = lidar_quat[0];
  lidar_tf.transform.rotation.y = lidar_quat[1];
  lidar_tf.transform.rotation.z = lidar_quat[2];
  lidar_tf.transform.rotation.w = lidar_quat[3];
  tf_broadcaster_->sendTransform(lidar_tf);


  odom_msg.header.stamp = lidar_tf.header.stamp;
  odom_msg.header.frame_id ="map";
  odom_msg.child_frame_id = "base_link";

  odom_msg.pose.pose.position.x = lidar_tf.transform.translation.x;
  odom_msg.pose.pose.position.y = lidar_tf.transform.translation.y;
  odom_msg.pose.pose.position.z = lidar_tf.transform.translation.z;

  odom_msg.pose.pose.orientation.x = lidar_tf.transform.rotation.x;
  odom_msg.pose.pose.orientation.y = lidar_tf.transform.rotation.y;
  odom_msg.pose.pose.orientation.z = lidar_tf.transform.rotation.z;
  odom_msg.pose.pose.orientation.w = lidar_tf.transform.rotation.w;

  publisher_odom_->publish(odom_msg);

  is_set = true;
}


void InfraStaticTFNode::timer_callback(){
  pole_tf.header.stamp = this->get_clock()->now();
  tf_broadcaster_->sendTransform(pole_tf);

  lidar_tf.header.stamp = pole_tf.header.stamp;
  tf_broadcaster_->sendTransform(lidar_tf);

  odom_msg.header.stamp = pole_tf.header.stamp;
  publisher_odom_->publish(odom_msg);
}

void InfraStaticTFNode::map_timer_callback(){
  world_tf.header.stamp = this->get_clock()->now();
  tf_broadcaster_->sendTransform(world_tf);

  pole_tf.header.stamp = world_tf.header.stamp;
  tf_broadcaster_->sendTransform(pole_tf);

  lidar_tf.header.stamp = world_tf.header.stamp;
  tf_broadcaster_->sendTransform(lidar_tf);

  odom_msg.header.stamp = world_tf.header.stamp;
  publisher_odom_->publish(odom_msg);
}
