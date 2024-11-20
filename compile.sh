#!/bin/bash

# Navigate to the root of the workspace
cd "$(dirname "$0")/../.."

# Build the package
colcon build --packages-select $(basename "$(dirname "$0")")

# Source the workspace
source /home/guojun/ros2_ws/install/setup.zsh

echo "Build and source complete. You can now run your package."
