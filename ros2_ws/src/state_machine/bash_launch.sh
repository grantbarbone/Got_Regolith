#!/bin/bash

# Navigate to the ZED SDK tutorial build directory
cd "/home/rover/Downloads/zed-sdk/tutorials/tutorial 1 - hello ZED/cpp/build" || { echo "Failed to change directory"; exit 1; }

# Check if the argument is 1
if [ "$1" -eq 1 ]; then
    # Clean the build directory
    rm -rf *

    # Run cmake and build the project
    cmake .. || { echo "CMake failed"; exit 1; }
    cmake --build . || { echo "Build failed"; exit 1; }
fi

# Navigate to the ROS 2 workspace directory
cd "/home/rover/ros2_ws" || { echo "Failed to change directory to ROS 2 workspace"; exit 1; }

# Check if the argument is 1
if [ "$1" -eq 1 ]; then
    # Build the ROS 2 workspace
    colcon build || { echo "ROS 2 build failed"; exit 1; }
fi

# Source the necessary setup scripts
source /home/rover/ros2_ws/install/setup.bash
source /opt/ros/humble/setup.bash

# Run both the ZED tutorial and ROS 2 node in parallel
#"/home/rover/Downloads/zed-sdk/tutorials/tutorial 1 - hello ZED/cpp/build/ZED_Tutorial_1" &  # Run in background
ros2 run state_machine main_subpub #&  # Run in background

# Wait for both processes to finish
