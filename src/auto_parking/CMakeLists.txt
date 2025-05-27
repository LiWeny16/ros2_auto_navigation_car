cmake_minimum_required(VERSION 3.8)
project(auto_parking)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(auto_msgs REQUIRED)
find_package(auto_simulation REQUIRED)

# 查找JSON库
find_package(nlohmann_json REQUIRED)

# 创建自动泊车仿真库
add_library(auto_parking_lib
  src/parking_simulator_node.cpp
  src/interface_adapters/parking_mqtt_adapter.cpp
  src/use_cases/parking_planner.cpp
  src/frameworks/parking_visualization.cpp
)

target_include_directories(auto_parking_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(auto_parking_lib PUBLIC cxx_std_17)

target_link_libraries(auto_parking_lib
  nlohmann_json::nlohmann_json
)

ament_target_dependencies(
  auto_parking_lib
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  auto_msgs
  auto_simulation
)

# 创建自动泊车仿真节点
add_executable(parking_simulator_node
  src/parking_simulator_node.cpp
)

target_include_directories(parking_simulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(parking_simulator_node PUBLIC cxx_std_17)

target_link_libraries(parking_simulator_node
  auto_parking_lib
)

ament_target_dependencies(
  parking_simulator_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  auto_msgs
  auto_simulation
)

# 安装目标
install(TARGETS parking_simulator_node auto_parking_lib
  DESTINATION lib/${PROJECT_NAME})

# 安装头文件
install(DIRECTORY 
  entities/
  use_cases/
  interface_adapters/
  frameworks/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.hpp"
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  PATTERN ".svn" EXCLUDE
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  PATTERN ".svn" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
