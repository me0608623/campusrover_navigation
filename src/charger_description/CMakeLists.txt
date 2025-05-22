cmake_minimum_required(VERSION 3.5)
project(charger_description)

find_package(ament_cmake REQUIRED)

# 安裝 URDF、Meshes、Launch、其他資源到 share/
install(DIRECTORY
  urdf
  meshes
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
