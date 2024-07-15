cmake_minimum_required(VERSION 3.8)
project(graph_algorithm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
)

# Add executable
add_executable(graph_algorithm_node src/graph_algorithm_node.cpp)

# Link libraries to the executable target
target_link_libraries(graph_algorithm_node
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${rcl_interfaces_LIBRARIES}
)

# Install targets
install(TARGETS
  graph_algorithm_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install mesh files
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Install mesh files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_export_dependencies(rosidl_default_runtime)

# Add testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
