cmake_minimum_required(VERSION 3.8)
project(state_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost 1.74 REQUIRED COMPONENTS filesystem system)
find_package(Threads REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)
find_package(visualization_msgs REQUIRED)
#find_package(Python3 REQUIRED COMPONENTS Development)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(main_subpub src/main.cpp)
target_link_libraries(main_subpub Python3::Python)
ament_target_dependencies(main_subpub rclcpp std_msgs)


add_executable(zed_subpub src/zed.cpp)
ament_target_dependencies(zed_subpub rclcpp std_msgs)

 #add_executable(graph_algorithm_subpub src/graph_algorithm.cpp)
 #ament_target_dependencies(graph_algorithm_subpub rclcpp std_msgs visualization_msgs)

 add_executable(simple_rviz2_node src/simple_rviz2_node.cpp)
 ament_target_dependencies(simple_rviz2_node rclcpp visualization_msgs)

add_executable(motor_controls_subpub src/motor_controls.cpp)
target_link_libraries(motor_controls_subpub Threads::Threads)
ament_target_dependencies(motor_controls_subpub rclcpp std_msgs boost_system)

add_executable(battery_subpub src/battery.cpp)
ament_target_dependencies(battery_subpub rclcpp std_msgs)

add_executable(lidar_subpub src/lidar.cpp)
ament_target_dependencies(lidar_subpub rclcpp std_msgs)

# add_executable(radar_subpub src/radar.cpp)
# ament_target_dependencies(radar_subpub rclcpp std_msgs)

# add_executable(topic_main_to_motor_controls src/topic_main_to_motor_controls.cpp)
# ament_target_dependencies(topic_main_to_motor_controls rclcpp std_msgs)

# add_executable(publisher src/publisher.cpp)
# ament_target_dependencies(publisher rclcpp std_msgs)

install(TARGETS
  main_subpub
  zed_subpub
  lidar_subpub
  simple_rviz2_node
  # graph_algorithm_subpub
  motor_controls_subpub
  battery_subpub
  #radar_subpub
  # topic_main_to_motor_controls
  # publisher
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(
  PROGRAMS
    /home/parallels/ros2_ws/src/python_applications/python_applications/graph_algorithm.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
