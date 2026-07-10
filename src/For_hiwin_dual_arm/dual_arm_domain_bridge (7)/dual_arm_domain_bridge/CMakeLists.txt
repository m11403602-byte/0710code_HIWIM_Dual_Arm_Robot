# dual_arm_domain_bridge (Plan B 兩支橋: 上行合併 + 下行端點透明轉送)
cmake_minimum_required(VERSION 3.8)
project(dual_arm_domain_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)

# 程式 1: 上行橋 (常駐)
add_executable(joint_state_uplink_bridge src/joint_state_uplink_bridge.cpp)
target_include_directories(joint_state_uplink_bridge PUBLIC include)
ament_target_dependencies(joint_state_uplink_bridge rclcpp sensor_msgs)

# 程式 2: 下行端點透明轉送
add_executable(trajectory_downlink_endpoint_relay src/trajectory_downlink_endpoint_relay.cpp)
target_include_directories(trajectory_downlink_endpoint_relay PUBLIC include)
ament_target_dependencies(trajectory_downlink_endpoint_relay
  rclcpp rclcpp_action control_msgs)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS joint_state_uplink_bridge trajectory_downlink_endpoint_relay
  DESTINATION lib/${PROJECT_NAME})

ament_package()
