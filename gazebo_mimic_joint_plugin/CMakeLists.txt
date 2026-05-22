cmake_minimum_required(VERSION 3.8)
project(gazebo_mimic_joint_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

add_library(gazebo_mimic_joint_plugin SHARED
  src/mimic_joint_plugin.cpp
)
target_include_directories(gazebo_mimic_joint_plugin PUBLIC
  ${gazebo_dev_INCLUDE_DIRS}
)
target_link_libraries(gazebo_mimic_joint_plugin
  ${gazebo_dev_LIBRARIES}
)
ament_target_dependencies(gazebo_mimic_joint_plugin
  gazebo_dev
  gazebo_ros
)

install(TARGETS gazebo_mimic_joint_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
