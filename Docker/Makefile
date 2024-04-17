SHELL = /bin/bash

build:
	colcon build

run:
	. ./install/setup.bash && ros2 run typefly robot

run-c:
	. ./install/setup.bash && ros2 run typefly controller

launch-turtle:
	ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

launch: launch-nav2 launch-slam launch-rviz

launch-nav2:
	ros2 launch nav2_bringup navigation_launch.py &

launch-slam:
	ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./slam_config.yaml &

launch-rviz:
	ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz &