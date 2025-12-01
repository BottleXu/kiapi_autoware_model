cmake_minimum_required(VERSION 3.8)
project(kiapi_perception_launch)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)

# find_package(rclcpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(autoware_map_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED components common)
find_package(pcl_conversions REQUIRED)
find_package (GeographicLib REQUIRED)
find_package(yaml-cpp REQUIRED) 

ament_auto_find_build_dependencies()

include_directories(
  include
)

add_library(pcl_bypass_component SHARED
  src/pointcloud_bypass.cpp
)

target_compile_definitions(pcl_bypass_component 
PRIVATE "KIAPI_PERCEPTION_LAUNCH_BUILDING_DLL")

ament_target_dependencies(pcl_bypass_component 
  rclcpp
  rclcpp_components
  sensor_msgs
  PCL
  pcl_conversions
)

rclcpp_components_register_nodes(pcl_bypass_component 
  "kiapi_perception_launch::PCLBypassComponent")

  install(TARGETS
  pcl_bypass_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

set(node_plugins "$(node_plugins)kiapi_perception_launch::PCLBypassComponent;$<TARGET_FILE:pcl_bypass_component>\n")

add_executable(pcl_bypass_node 
  src/pointcloud_bypass_node.cpp)

ament_target_dependencies(pcl_bypass_node
  rclcpp
  rclcpp_components
  sensor_msgs
  PCL
  pcl_conversions
)

install(TARGETS 
  pcl_bypass_node 
  DESTINATION lib/${PROJECT_NAME}
)

# pointcloud_xyzirt_bypass_node
add_executable(pointcloud_xyzirt_bypass_node 
  src/pointcloud_xyzirt_bypass_node.cpp)

ament_target_dependencies(pointcloud_xyzirt_bypass_node
  rclcpp
  rclcpp_components
  sensor_msgs
  PCL
  pcl_conversions
)

# pointcloud_xyzitr_bypass_node
add_executable(pointcloud_xyzitr_bypass_node 
  src/pointcloud_xyzitr_bypass_node.cpp)

ament_target_dependencies(pointcloud_xyzitr_bypass_node
  rclcpp
  rclcpp_components
  sensor_msgs
  PCL
  pcl_conversions
)

# infra_static_tf_node
add_executable(infra_static_tf_node 
  src/infra_static_tf_node.cpp)

ament_target_dependencies(infra_static_tf_node
  rclcpp
  rclcpp_components
  nav_msgs
  rclcpp
  tf2
  tf2_ros
  GeographicLib
  autoware_map_msgs
)
target_link_libraries(infra_static_tf_node
  GeographicLib
  yaml-cpp
)

# pose converter
add_executable(utm_tracker 
  src/utm_tracker.cpp)

ament_target_dependencies(utm_tracker
  rclcpp
  rclcpp_components
  nav_msgs
  geographic_msgs
  rclcpp
  tf2
  tf2_ros
  autoware_map_msgs
  autoware_perception_msgs
)

# cpp install
install(TARGETS 
  pointcloud_xyzirt_bypass_node
  pointcloud_xyzitr_bypass_node
  infra_static_tf_node 
  utm_tracker
  DESTINATION lib/${PROJECT_NAME}
)


ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)

# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/config
# )