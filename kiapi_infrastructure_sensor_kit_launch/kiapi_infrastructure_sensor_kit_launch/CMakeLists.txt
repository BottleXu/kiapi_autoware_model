cmake_minimum_required(VERSION 3.5)
project(kiapi_infrastructure_sensor_kit_launch)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## Add include directories
include_directories(include)

set(node_plugins "")

# Component library
add_library(pcl_sync_component SHARED
  src/infra_node.cpp
)
target_compile_definitions(pcl_sync_component
PRIVATE "KIAPI_INFRASTRUCTURE_SENSOR_KIT_LAUNCH_BUILDING_DLL")

ament_target_dependencies(pcl_sync_component
  rclcpp
  rclcpp_components
  sensor_msgs
)

rclcpp_components_register_nodes(pcl_sync_component "kiapi_infrastructure::PCLSyncNode")
set(node_plugins "$(node_plugins)kiapi_infrastructure::PCLSyncNode;$<TARGET_FILE:pcl_sync_component>\n")

install(TARGETS
  pcl_sync_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Install Python executables
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/infra_sensor.py
  ${PROJECT_NAME}/pointcloud_sync.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install other shared assets
ament_auto_package(INSTALL_TO_SHARE
  launch
  data
  config
)



# ament_package()