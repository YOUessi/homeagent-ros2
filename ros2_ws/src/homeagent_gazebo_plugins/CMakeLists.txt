cmake_minimum_required(VERSION 3.8)
project(homeagent_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)

add_library(homeagent_contact_grasp_plugin SHARED
  src/contact_grasp_plugin.cpp
)
target_include_directories(homeagent_contact_grasp_plugin PRIVATE
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_directories(homeagent_contact_grasp_plugin PRIVATE
  ${GAZEBO_LIBRARY_DIRS}
)
target_link_libraries(homeagent_contact_grasp_plugin
  ${GAZEBO_LIBRARIES}
)
target_compile_options(homeagent_contact_grasp_plugin PRIVATE
  ${GAZEBO_CXX_FLAGS}
)

install(TARGETS homeagent_contact_grasp_plugin
  LIBRARY DESTINATION lib
)

ament_package()
