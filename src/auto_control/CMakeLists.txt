cmake_minimum_required(VERSION 3.8)
project(auto_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(auto_msgs REQUIRED)

# 添加纯跟踪控制器库
add_library(pure_pursuit_controller
  src/pure_pursuit_controller.cpp
)
target_include_directories(pure_pursuit_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pure_pursuit_controller PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  pure_pursuit_controller
  "rclcpp"
  "geometry_msgs"
  "auto_msgs"
  "tf2"
  "tf2_geometry_msgs"
)

# 添加控制节点
add_executable(controller_node src/controller_node.cpp)
target_include_directories(controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(controller_node
  pure_pursuit_controller
)
ament_target_dependencies(
  controller_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "visualization_msgs"
  "auto_msgs"
)

install(TARGETS controller_node pure_pursuit_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

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

  find_package(ament_cmake_gtest REQUIRED)

  # Add controller tests
  ament_add_gtest(test_controller test/test_controller.cpp)
  target_link_libraries(test_controller
    pure_pursuit_controller
  )
  ament_target_dependencies(test_controller
    "rclcpp"
    "auto_msgs"
    "geometry_msgs"
    "tf2"
    "tf2_geometry_msgs"
  )
endif()

ament_package()
