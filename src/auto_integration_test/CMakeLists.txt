cmake_minimum_required(VERSION 3.8)
project(auto_integration_test)

# 默认使用C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(auto_msgs REQUIRED)
find_package(auto_perception REQUIRED)
find_package(auto_planning REQUIRED)
find_package(auto_control REQUIRED)
find_package(auto_simulation REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 包含头文件目录
include_directories(include)

# 添加系统测试运行器库
add_library(system_test_runner 
  src/system_test_runner.cpp
)
ament_target_dependencies(system_test_runner
  rclcpp
  auto_msgs
  auto_perception
  auto_planning
  auto_control
  auto_simulation
  visualization_msgs
  geometry_msgs
  std_msgs
)

# 添加性能优化器库
add_library(performance_optimizer 
  src/performance_optimizer.cpp
)
ament_target_dependencies(performance_optimizer
  rclcpp
  auto_msgs
  auto_perception
  auto_planning
  auto_control
  auto_simulation
  visualization_msgs
  geometry_msgs
  std_msgs
)

# 添加错误处理器库
add_library(error_handler 
  src/error_handler.cpp
)
ament_target_dependencies(error_handler
  rclcpp
)

# 添加系统测试运行器可执行文件
add_executable(test_runner src/test_runner_main.cpp)
target_link_libraries(test_runner system_test_runner)
ament_target_dependencies(test_runner
  rclcpp
)

# 添加性能优化器可执行文件
add_executable(optimizer src/optimizer_main.cpp)
target_link_libraries(optimizer performance_optimizer)
ament_target_dependencies(optimizer
  rclcpp
)

# 安装目标
install(TARGETS
  system_test_runner
  performance_optimizer
  error_handler
  test_runner
  optimizer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_export_include_directories(include)
ament_export_libraries(system_test_runner performance_optimizer error_handler)
ament_export_dependencies(
  rclcpp
  auto_msgs
  auto_perception
  auto_planning
  auto_control
  auto_simulation
)

ament_package()
