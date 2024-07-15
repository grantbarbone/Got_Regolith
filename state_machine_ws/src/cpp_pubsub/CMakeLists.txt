cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Set Boost options
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem regex chrono)

set(dependencies
  rclcpp
  std_msgs
  custom_msgs
  rcl_interfaces
)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
# Include directories
include_directories(
    PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS}
)


# Create executables
add_executable(sensor_state_machine src/sensor_state_machine.cpp)
add_executable(lidar_pubsub src/lidar_pubsub.cpp)
add_executable(graph_algorithm_pubsub src/graph_algorithm_pubsub.cpp)

ament_target_dependencies(sensor_state_machine ${dependencies})
ament_target_dependencies(lidar_pubsub ${dependencies})
ament_target_dependencies(graph_algorithm_pubsub ${dependencies})


# Link with libraries
target_link_libraries(sensor_state_machine
  ${Boost_LIBRARIES}
  Boost::chrono
  Boost::regex
)

target_link_libraries(lidar_pubsub
  ${Boost_LIBRARIES}  Boost::chrono
  Boost::chrono
  Boost::regex

)

target_link_libraries(graph_algorithm_pubsub
  ${Boost_LIBRARIES}
  Boost::chrono
  Boost::regex
)

# Install targets
install(TARGETS
  sensor_state_machine
  lidar_pubsub
  graph_algorithm_pubsub
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include files if needed
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_dependencies(rosidl_default_runtime)

# Test dependencies (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
