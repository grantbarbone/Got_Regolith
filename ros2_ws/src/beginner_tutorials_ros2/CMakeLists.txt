cmake_minimum_required(VERSION 3.5)
project(beginner_tutorials)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# for the publisher
add_executable(main_subpub src/main.cpp)
ament_target_dependencies(main_subpub rclcpp std_msgs)

add_executable(zed_subpub src/zed.cpp)
ament_target_dependencies(zed_subpub rclcpp std_msgs)

add_executable(graph_algorithm_subpub src/graph_algorithm.cpp)
ament_target_dependencies(graph_algorithm_subpub rclcpp std_msgs)

add_executable(motor_controls_subpub src/motor_controls.cpp)
ament_target_dependencies(motor_controls_subpub rclcpp std_msgs)

add_executable(battery_subpub src/battery.cpp)
ament_target_dependencies(battery_subpub rclcpp std_msgs)

add_executable(lidar_subpub src/lidar.cpp)
ament_target_dependencies(lidar_subpub rclcpp std_msgs)

add_executable(radar_subpub src/radar.cpp)
ament_target_dependencies(radar_subpub rclcpp std_msgs)

install(TARGETS
  main_subpub
  zed_subpub
  lidar_subpub
  graph_algorithm_subpub
  motor_controls_subpub
  battery_subpub
  radar_subpub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
