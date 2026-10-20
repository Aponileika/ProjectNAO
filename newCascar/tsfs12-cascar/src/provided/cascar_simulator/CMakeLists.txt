cmake_minimum_required(VERSION 3.8)
project(cascar_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cascar_interfaces REQUIRED)
find_package(tf2 REQUIRED)

add_executable(simulator src/simulator.cpp)
ament_target_dependencies(simulator rclcpp tf2 nav_msgs sensor_msgs cascar_interfaces)

install(TARGETS
        simulator
        DESTINATION lib/${PROJECT_NAME})

ament_package()
