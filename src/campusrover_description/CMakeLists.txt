cmake_minimum_required(VERSION 3.5)
project(campusrover_description)

find_package(ament_cmake REQUIRED)

# 安裝 URDF、Meshes、Launch、RViz 資料夾到 share/
install(DIRECTORY
  urdf
  meshes
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
