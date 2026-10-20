cmake_minimum_required(VERSION 3.8)
project(cascar_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(odom_tf_publisher src/odom_tf_publisher.cpp)
ament_target_dependencies(odom_tf_publisher rclcpp tf2 tf2_ros nav_msgs geometry_msgs)

install(TARGETS
        odom_tf_publisher
        DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
