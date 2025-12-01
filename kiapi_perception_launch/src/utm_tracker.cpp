#include "utm_tracker.hpp"

UTMTracker::UTMTracker(const rclcpp::NodeOptions & options)
: Node("kiapi_object_pose_convert", options){
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    trackers_pub_ =  this->create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
        "/perception/object_recognition/tracking/objects",
         rclcpp::QoS{1});
    trackers_sub_ = create_subscription<autoware_perception_msgs::msg::TrackedObjects>(
         "/perception/object_recognition/tracking/objects/raw",
        rclcpp::QoS{1},
        std::bind(&UTMTracker::TrackedObjectsCallback, this, std::placeholders::_1));
    mapinfo_sub_ = create_subscription<autoware_map_msgs::msg::MapProjectorInfo>(
         "/map/map_projector_info",
        rclcpp::QoS{1},
        std::bind(&UTMTracker::MapinfoCallback, this, std::placeholders::_1));
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UTMTracker>());
  rclcpp::shutdown();
  return 0;
}
