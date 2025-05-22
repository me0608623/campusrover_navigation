cmake_minimum_required(VERSION 3.5)
project(campusrover_move)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find ament and ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(campusrover_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(costmap_converter_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# Service definitions
set(srv_files
  "srv/PullOverPathGenerator.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

# include headers
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# path_following node
add_executable(path_following src/path_following.cpp)
ament_target_dependencies(path_following
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  laser_geometry
  campusrover_msgs
  visualization_msgs
  costmap_converter_msgs
)

# 創建與 costmap_converter_msgs 的接口連接
rosidl_target_interfaces(path_following
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# dwa_planner node
add_executable(dwa_planner src/dwa_planner.cpp)
target_link_libraries(dwa_planner
  ${std_srvs_LIBRARIES}
)
ament_target_dependencies(dwa_planner
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  laser_geometry
  control_toolbox
  campusrover_msgs
  visualization_msgs
  costmap_converter_msgs
)

rosidl_target_interfaces(dwa_planner
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_include_directories(dwa_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

# # pullover_path_planner node
# add_executable(pullover_path_planner src/pullover_path_planner.cpp)
# ament_target_dependencies(pullover_path_planner
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   sensor_msgs
#   std_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
#   laser_geometry
#   campusrover_msgs
# )

# install binaries
install(TARGETS
  path_following
  dwa_planner
  # pullover_path_planner
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# install srv files so ros2 interface show works
install(
  DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
