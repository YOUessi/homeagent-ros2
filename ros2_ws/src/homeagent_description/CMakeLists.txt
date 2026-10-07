cmake_minimum_required(VERSION 3.8)
project(homeagent_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
