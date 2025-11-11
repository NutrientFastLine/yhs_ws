cmake_minimum_required(VERSION 3.8)
project(turn_on_yhs_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)  # Nav2速度消息依赖
find_package(yhs_can_interfaces REQUIRED)  # 底盘控制消息依赖

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

# 编译节点
add_executable(nav2_chassis_adapter src/nav2_chassis_adapter.cpp)
ament_target_dependencies(nav2_chassis_adapter
  rclcpp
  geometry_msgs
  yhs_can_interfaces
)

# 安装节点
install(TARGETS
  nav2_chassis_adapter
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
