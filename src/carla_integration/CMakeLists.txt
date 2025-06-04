cmake_minimum_required(VERSION 3.8)
project(carla_integration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(auto_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# PCL dependencies (optional for now)
# find_package(pcl_ros REQUIRED)
# find_package(pcl_conversions REQUIRED)

# CARLA bridge dependencies (optional for now)
# find_package(carla_msgs REQUIRED)

# Include directories
include_directories(include)

# CARLA Map Adapter
add_executable(carla_map_adapter 
  src/carla_map_adapter.cpp
)

ament_target_dependencies(carla_map_adapter
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  auto_msgs
)

# CARLA Sensor Adapter
add_executable(carla_sensor_adapter 
  src/carla_sensor_adapter.cpp
)

ament_target_dependencies(carla_sensor_adapter
  rclcpp
  std_msgs
  sensor_msgs
  auto_msgs
)

# CARLA Control Adapter
add_executable(carla_control_adapter 
  src/carla_control_adapter.cpp
)

ament_target_dependencies(carla_control_adapter
  rclcpp
  std_msgs
  geometry_msgs
  auto_msgs
)

# Install targets
install(TARGETS
  carla_map_adapter
  carla_sensor_adapter
  carla_control_adapter
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
