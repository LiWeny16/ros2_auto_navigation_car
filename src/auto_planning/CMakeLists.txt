cmake_minimum_required(VERSION 3.8)
project(auto_planning)

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
find_package(tf2_geometry_msgs REQUIRED)
find_package(auto_perception REQUIRED)

# Add A* planner library
add_library(a_star_planner
  src/a_star_planner.cpp
)
target_include_directories(a_star_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(a_star_planner PUBLIC cxx_std_17)
ament_target_dependencies(a_star_planner
  rclcpp
  auto_msgs
  geometry_msgs
)

# Add Optimized A* planner library
add_library(optimized_a_star_planner
  src/optimized_a_star_planner.cpp
)
target_include_directories(optimized_a_star_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(optimized_a_star_planner PUBLIC cxx_std_17)
ament_target_dependencies(optimized_a_star_planner
  rclcpp
  auto_msgs
  geometry_msgs
)

# Add Hybrid A* planner library
add_library(hybrid_a_star_planner
  src/hybrid_a_star_planner.cpp
)
target_include_directories(hybrid_a_star_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(hybrid_a_star_planner PUBLIC cxx_std_17)
ament_target_dependencies(hybrid_a_star_planner
  rclcpp
  auto_msgs
  geometry_msgs
)

# Add decision maker library
add_library(decision_maker
  src/decision_maker.cpp
)
target_include_directories(decision_maker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(decision_maker PUBLIC cxx_std_17)
ament_target_dependencies(decision_maker
  rclcpp
  auto_msgs
  geometry_msgs
  auto_perception
)

# Add path planner node executable
add_executable(path_planner_node src/path_planner_node.cpp)
target_include_directories(path_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(path_planner_node PUBLIC cxx_std_17)
target_link_libraries(path_planner_node
  a_star_planner
  hybrid_a_star_planner
  decision_maker
)
ament_target_dependencies(
  path_planner_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  auto_msgs
  tf2_geometry_msgs
  auto_perception
)

# Add decision visualization node executable
add_executable(decision_visualization_node src/decision_visualization_node.cpp)
target_include_directories(decision_visualization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(decision_visualization_node PUBLIC cxx_std_17)
ament_target_dependencies(
  decision_visualization_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  auto_msgs
  tf2_geometry_msgs
)

# Install targets
install(TARGETS 
  path_planner_node
  decision_visualization_node
  a_star_planner
  optimized_a_star_planner
  hybrid_a_star_planner
  decision_maker
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files if any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  PATTERN ".svn" EXCLUDE
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
  
  find_package(ament_cmake_gtest REQUIRED)
  
  # Add planner tests
  ament_add_gtest(test_planner test/test_planner.cpp)
  target_link_libraries(test_planner
    a_star_planner
    optimized_a_star_planner
    hybrid_a_star_planner
  )
  ament_target_dependencies(test_planner
    "rclcpp"
    "auto_msgs"
    "geometry_msgs"
    "tf2"
    "tf2_geometry_msgs"
  )
endif()

ament_package()
