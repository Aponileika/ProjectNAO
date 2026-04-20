cmake_minimum_required(VERSION 3.8)
project(cascar_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/CarCommand.msg"
        "msg/CarMeasurement.msg"
        "msg/CarTicks.msg"
        "msg/ControlRequest.msg"
        "msg/Points2D.msg"
        "msg/NodeIDs.msg"
        "msg/IDMCommand.msg"
        "msg/MissionControl.msg"
        "msg/SensorBias.msg"
        "msg/SteeringAngle.msg"
        "srv/SetState.srv"
        "srv/NodeIDs.srv"
        "srv/Points2D.srv"
        DEPENDENCIES builtin_interfaces std_msgs
        )

ament_package()
