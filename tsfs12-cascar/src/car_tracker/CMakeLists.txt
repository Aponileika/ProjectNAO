cmake_minimum_required(VERSION 3.8)
project(car_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(car_tracker src/car_tracker.cpp)
ament_target_dependencies(car_tracker rclcpp std_srvs tf2 tf2_ros nav_msgs geometry_msgs)


install(TARGETS
        car_tracker
        DESTINATION lib/${PROJECT_NAME})

ament_package()
