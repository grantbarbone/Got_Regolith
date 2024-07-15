cmake_minimum_required(VERSION 3.5)
project(custom_msgs)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Add message files
set(msg_files
  "msg/LidarData.msg"
  "msg/StereoCameraData.msg"
  "msg/RadarData.msg"
  "msg/GraphAlgorithmData.msg"
  "msg/MotorControls.msg"
)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)
  
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME}/   # Directory where headers are located after generation
  DESTINATION include/${PROJECT_NAME}/   # Destination directory in install space
)

#Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()
