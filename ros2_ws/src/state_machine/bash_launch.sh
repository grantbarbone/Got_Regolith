#!/bin/bash

# Navigate to the ZED SDK tutorial build directory
cd "/home/rover/Downloads/zed-sdk/tutorials/tutorial 1 - hello ZED/cpp/build" || exit

# Check if the argument is 1
if [ "$1" -eq 1 ]; then
    # Clean the build directory
    rm -rf *

    # Run cmake and build the project
    cmake ..
    cmake --build .
fi

# Run the ZED tutorial executable
./ZED_Tutorial_1

# Navigate to the ROS 2 workspace directory
cd "/home/rover/ros2_ws" || exit

# Check if the argument is 1
if [ "$1" -eq 1 ]; then
    # Build the ROS 2 workspace
    colcon build
fi
ros2 run state_machine main_subpub
