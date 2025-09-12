#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <math.h>
#include <vector>
#include <memory>

#include <GeographicLib/TransverseMercator.hpp>

//resource from wiki  Universal Transverse Mercator coordinate system. WGS 84
enum Hemi {NorthH, SouthH};


struct basic{
  double bx = 0.0;//-594929.9431329881;//east
  double by = 0.0;//-4139043.529676078;//north
  double bz = 0.0;//unit in m
};//origin point

class UTMtoLLANode : public rclcpp::Node
{
public:
  explicit UTMtoLLANode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
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
  


//   GeographicLib::TransverseMercator tm_;  //tm need an initial value for constructor:  at(earth radius),fla(inverse flattening), k0. if I define at, fla and k0 as private member and use it here then the compiler will show error. I need assign the value from geo_ToLLA constructor.
  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_gnss_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;

  // TF broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_path_;


  void UTMConvert2LLA(Hemi hemi,int zone, double east, double north, double height);
  void LLAConvert2UTM(Hemi hemi, int zone, double latitude, double longitude, double altitude);
  void WorldToMap(void);

  // Callbacks
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void pub_tf(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  // Quaternion storage
  double qx_ = 0.0, qy_ = 0.0, qz_ = 0.0, qw_ = 1.0;

  // Path storage
  nav_msgs::msg::Path path_;

  size_t tmp_counter = 0;
  double tmp_x, tmp_y, tmp_z;
  std::vector<double> his_x,his_y,his_z;
  int zone_;
  std::string hemi_;

  int64_t maximum_queue_size_ = 10;
};

void UTMtoLLANode::SetValues(std::string hemi, int zone, double at, double fla, double k0)
{
    hemi_ = hemi;
    zone_ = zone;
    tm_ = std::make_unique<GeographicLib::TransverseMercator>(at, fla, k0);
}

void UTMtoLLANode::UTMConvert2LLA(Hemi hemi,int zone, double east, double north, double height)
{
  double latitude;
  double longitude;

  int lon0   = zone*6-183;
  east  -= kE0_;
  north -= (hemi==NorthH) ? kNN_ : kNS_; 

  tm_->Reverse(lon0, east, north, latitude, longitude); 

  lla_.push_back(latitude); lla_.push_back(longitude); lla_.push_back(height);
}

void UTMtoLLANode::LLAConvert2UTM(Hemi hemi, int zone, double latitude, double longitude, double height){
  double east,north;

  int lon0  = zone*6-183;

  tm_->Forward(lon0, latitude, longitude, east, north);

  east += kE0_;
  north += (hemi==NorthH) ? kNN_ : kNS_;

  utm_.push_back(east); utm_.push_back(north); utm_.push_back(height);
}

std::vector<double> UTMtoLLANode::get_lla()
{
  return lla_;
}

void UTMtoLLANode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  qx_ = msg->orientation.x;
  qy_ = msg->orientation.y;
  qz_ = msg->orientation.z;
  qw_ = msg->orientation.w;
}

void UTMtoLLANode::WorldToMap(void)
{

  int lon0_map  = zone_*6-183;
  double map_east, map_north;
  double map_latitude = 35.647624;
  double map_longitude = 128.400457;

  tm_->Forward(lon0_map, map_latitude, map_longitude, map_east, map_north);

  map_east += kE0_;
  map_north += kNN_;

  world_utm_.push_back(map_east); world_utm_.push_back(map_north); world_utm_.push_back(0.0);
}

void UTMtoLLANode::pub_tf(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{   
  basic bs;

  //world to map

  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header.stamp = msg->header.stamp;
  // tf_msg.header.frame_id = "map";
  tf_msg.header.frame_id = "world";
  // tf_msg.child_frame_id = "utm_to_lla_link";
  tf_msg.child_frame_id = "map";

  tf_msg.transform.translation.x = 445724.933;
  tf_msg.transform.translation.y = 3945031.452;
  tf_msg.transform.translation.z = 0;

  tf_msg.transform.rotation.x = 0;
  tf_msg.transform.rotation.y = 0;
  tf_msg.transform.rotation.z = 0;
  tf_msg.transform.rotation.w = 1;

  tf_broadcaster_->sendTransform(tf_msg);


  if (!tm_) {
  RCLCPP_ERROR(this->get_logger(), "TransverseMercator not initialized. Call SetValues first.");
  return;}

  if(hemi_ == "North")
    LLAConvert2UTM(NorthH, zone_, msg->latitude, msg->longitude, msg->altitude);
  else if(hemi_ == "South")
    LLAConvert2UTM(SouthH, zone_, msg->latitude, msg->longitude, msg->altitude);
  else {
    RCLCPP_ERROR(this->get_logger(), "Invalid hemisphere: %s", hemi_.c_str());
    return;}

  if (utm_.size() < 3) {
    RCLCPP_WARN(this->get_logger(), "UTM data not available yet");
    return;
    }

  //world to utm
  // geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header.stamp = msg->header.stamp;
  // tf_msg.header.frame_id = "map";
  tf_msg.header.frame_id = "world";
  // tf_msg.child_frame_id = "utm_to_lla_link";
  tf_msg.child_frame_id = "base_link";

  tf_msg.transform.translation.x = utm_[0] + bs.bx;
  tf_msg.transform.translation.y = utm_[1] + bs.by;
  tf_msg.transform.translation.z = utm_[2] + bs.bz;

  tf_msg.transform.rotation.x = qx_;
  tf_msg.transform.rotation.y = qy_;
  tf_msg.transform.rotation.z = qz_;
  tf_msg.transform.rotation.w = qw_;

  tf_broadcaster_->sendTransform(tf_msg);

  utm_.clear();

  
  // Path Pub
  geometry_msgs::msg::PoseStamped poses;
  poses.header.stamp = msg->header.stamp;
  // poses.header.frame_id = "map";
  poses.header.frame_id = "world";


  poses.pose.position.x =  tf_msg.transform.translation.x;
  poses.pose.position.y =  tf_msg.transform.translation.y;
  poses.pose.position.z =  tf_msg.transform.translation.z;

  poses.pose.orientation = tf_msg.transform.rotation;

  // path_.header.frame_id = "map";
  path_.header.frame_id = "world";
  path_.header.stamp = msg->header.stamp;
  path_.poses.push_back(poses);

  publisher_path_->publish(path_);

}
