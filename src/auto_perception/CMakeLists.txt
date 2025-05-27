cmake_minimum_required(VERSION 3.8)
project(auto_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(auto_msgs REQUIRED)

# 添加对象检测器库
add_library(object_detector
  src/object_detector.cpp
)
target_include_directories(object_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(object_detector PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  object_detector
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "visualization_msgs"
  "auto_msgs"
)

# 添加感知节点
add_executable(perception_node src/perception_node.cpp)
target_include_directories(perception_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(perception_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(perception_node
  object_detector
)
ament_target_dependencies(
  perception_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "visualization_msgs"
  "tf2"
  "tf2_ros"
  "nav_msgs"
  "auto_msgs"
)

install(TARGETS perception_node object_detector
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)
  
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
#   PATTERN "*.py" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
#   PATTERN "*.launch*" PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
# )

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
