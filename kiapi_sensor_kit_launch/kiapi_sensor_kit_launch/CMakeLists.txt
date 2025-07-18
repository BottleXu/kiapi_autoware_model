cmake_minimum_required(VERSION 3.5)
project(kiapi_sensor_kit_launch)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)

# find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# # Include Cpp "include" directory
# include_directories(include)

# # Create Cpp executable
# add_executable(cpp_executable src/node.cpp)
# ament_target_dependencies(cpp_executable rclcpp)

# # Install Cpp executables
# install(TARGETS
#   cpp_executable
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install Python executables
install(PROGRAMS
  scripts/vehicle_info_from_sensor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  data
  config
)



