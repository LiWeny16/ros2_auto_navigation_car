cmake_minimum_required(VERSION 3.8)
project(auto_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(auto_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 查找MQTT库
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)
find_package(nlohmann_json REQUIRED)

# 添加MQTT桥接库
add_library(mqtt_bridge
  src/mqtt_bridge.cpp
)
target_include_directories(mqtt_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MOSQUITTO_INCLUDE_DIRS}
)
target_compile_features(mqtt_bridge PUBLIC cxx_std_17)
target_link_libraries(mqtt_bridge 
  ${MOSQUITTO_LIBRARIES}
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(
  mqtt_bridge
  rclcpp
  geometry_msgs
  auto_msgs
)

# 添加模拟节点
add_executable(simulation_node src/simulation_node.cpp)
target_include_directories(simulation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(simulation_node PUBLIC cxx_std_17)  # Require C++17
ament_target_dependencies(
  simulation_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  auto_msgs
  tf2
  tf2_geometry_msgs
)

# 添加MQTT桥接节点
add_executable(mqtt_bridge_node src/mqtt_bridge_node.cpp)
target_include_directories(mqtt_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MOSQUITTO_INCLUDE_DIRS}
)
target_compile_features(mqtt_bridge_node PUBLIC cxx_std_17)
target_link_libraries(mqtt_bridge_node
  mqtt_bridge
)
ament_target_dependencies(
  mqtt_bridge_node
  rclcpp
  std_msgs
  geometry_msgs
  auto_msgs
)

install(TARGETS simulation_node mqtt_bridge_node mqtt_bridge
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  PATTERN ".svn" EXCLUDE
)

# Install configuration files if any
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  PATTERN ".svn" EXCLUDE
)

# Install Python scripts
install(PROGRAMS
  scripts/websocket_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
