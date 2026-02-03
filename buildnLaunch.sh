#!/bin/bash
echo "Building the HMS_Perception package..."
colcon build --packages-select HMS_Perception

echo "Sourcing the workspace..."
source install/setup.bash

echo "Launching the rover in RVIZ..."
ros2 launch HMS_Perception display.launch.py
