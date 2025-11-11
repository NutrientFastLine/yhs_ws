cmake_minimum_required(VERSION 3.8)
project(yhs_nav2_launcher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

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

add_executable(trans_tf_2d src/trans_tf_2d.cpp)
ament_target_dependencies(trans_tf_2d rclcpp tf2_ros geometry_msgs)

install(TARGETS
  trans_tf_2d
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装配置文件
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装地图文件
install(
  DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

# 安装rviz配置文件
install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# 构建可执行节点
add_executable(static_body_to_base_link src/static_body_to_base_link.cpp)
ament_target_dependencies(static_body_to_base_link
  rclcpp
  tf2_ros
  geometry_msgs
)

# 安装可执行文件
install(TARGETS
  static_body_to_base_link
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
