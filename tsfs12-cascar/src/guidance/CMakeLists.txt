cmake_minimum_required(VERSION 3.8)
project(guidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cascar_msgs REQUIRED)

add_executable(guidance
  src/guidance.cpp
  src/hybridAStar.cpp
  src/dubinPath.cpp
)

target_include_directories(guidance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(guidance
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  cascar_msgs
)

install(TARGETS
  guidance
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()
