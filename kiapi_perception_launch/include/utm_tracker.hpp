// ROS2 Components
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "autoware_perception_msgs/msg/tracked_objects.hpp"
#include "autoware_map_msgs/msg/map_projector_info.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// enum Hemi {NorthH, SouthH};
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

class UTMTracker : public rclcpp::Node
{
  public:
    explicit UTMTracker(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  private:
    // TF broadcaster, Publisher
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};


    rclcpp::Subscription<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr trackers_sub_;
    rclcpp::Subscription<autoware_map_msgs::msg::MapProjectorInfo>::SharedPtr mapinfo_sub_;

    rclcpp::Publisher<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr trackers_pub_;

    // Conversion storage
    geometry_msgs::msg::TransformStamped world_tf;
    autoware_perception_msgs::msg::TrackedObjects local_TrackedObjects;
    autoware_perception_msgs::msg::TrackedObjects utm_TrackedObjects;
    geographic_msgs::msg::GeoPoint vector_map_point;

    lla_data vector_map_lla;
    utm_data vector_map_utm;

    // Callbacks
    void TrackedObjectsCallback(const autoware_perception_msgs::msg::TrackedObjects::SharedPtr msg);
    void MapinfoCallback(const autoware_map_msgs::msg::MapProjectorInfo::SharedPtr msg);
    void LookupTransform();
    void PoseConverter();

    bool is_map_info = false;

    // // LLAtoUTM
    // int zone_ = 52;
    // std::string hemi_ = "North";
    // std::unique_ptr<GeographicLib::TransverseMercator> tm_;
    // const double kNN_      = 0;
    // const double kNS_      = 10000000.0;
    // const double kE0_      = 500000.0;
    // const double kPI_      = 3.14159265359;
    // const double kDist_    = 1.2;

};

void UTMTracker::LookupTransform(){

    // Look up for the transformation between target_frame and turtle2 frames
    // and send velocity commands for turtle2 to reach target_frame
    try {
        world_tf = tf_buffer_->lookupTransform(
        "world", "map",
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO_ONCE(
        this->get_logger(), "Could not transform: %s",
        ex.what());
        trackers_pub_->publish(utm_TrackedObjects);
        return;
    }
    PoseConverter();

    // if (!is_map_info){
    //     trackers_pub_->publish(utm_TrackedObjects);
    // }
    // else{
    //     PoseConverter();
    // }
}

void UTMTracker::MapinfoCallback(const autoware_map_msgs::msg::MapProjectorInfo::SharedPtr msg){
    // vector_map_point = msg->map_origin;
    vector_map_lla.lat = msg->map_origin.latitude;
    vector_map_lla.lon = msg->map_origin.longitude;
    vector_map_lla.alt = msg->map_origin.altitude;
    is_map_info = true;
    RCLCPP_INFO(this->get_logger(), "MapinfoCallback: %d",is_map_info);
}



void UTMTracker::TrackedObjectsCallback(const autoware_perception_msgs::msg::TrackedObjects::SharedPtr msg){
    utm_TrackedObjects = *msg;
    LookupTransform();
}

void UTMTracker::PoseConverter(){
    int idx_i = 0;
    for (const autoware_perception_msgs::msg::TrackedObject &local_tracked_obj : utm_TrackedObjects.objects) {
        
        // double local_pose_x = local_tracked_obj.kinematics.pose_with_covariance.pose.position.x;
        // double local_pose_y = local_tracked_obj.kinematics.pose_with_covariance.pose.position.y;
        // double local_pose_z = local_tracked_obj.kinematics.pose_with_covariance.pose.position.z;

        // double local_quat_x = local_tracked_obj.kinematics.pose_with_covariance.pose.orientation.x;
        // double local_quat_y = local_tracked_obj.kinematics.pose_with_covariance.pose.orientation.y;
        // double local_quat_z = local_tracked_obj.kinematics.pose_with_covariance.pose.orientation.z;
        // double local_quat_w = local_tracked_obj.kinematics.pose_with_covariance.pose.orientation.w;

        utm_TrackedObjects.objects[idx_i].kinematics.pose_with_covariance.pose.position.x += world_tf.transform.translation.x;
        utm_TrackedObjects.objects[idx_i].kinematics.pose_with_covariance.pose.position.y += world_tf.transform.translation.y;
        // utm_TrackedObjects.objects[idx_i].kinematics.pose_with_covariance.pose.position.z;


        idx_i += 1;
    }
    utm_TrackedObjects.header.frame_id = "world";
    trackers_pub_->publish(utm_TrackedObjects);

}
