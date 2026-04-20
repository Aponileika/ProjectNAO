cmake_minimum_required(VERSION 3.8)
project(state_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cascar_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(state_publisher src/state_publisher.cpp)
ament_target_dependencies(state_publisher rclcpp tf2 tf2_ros nav_msgs sensor_msgs geometry_msgs cascar_msgs)

install(TARGETS
        state_publisher
        DESTINATION lib/${PROJECT_NAME})

ament_package()
