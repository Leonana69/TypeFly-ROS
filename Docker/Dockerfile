FROM ros:iron-ros-base-jammy

RUN apt update && apt install nano -y
RUN echo 'source ros_entrypoint.sh' >> ~/.bashrc

RUN apt install ros-iron-slam-toolbox -y
RUN apt install ros-iron-navigation2 -y
RUN apt install ros-iron-nav2-bringup -y
RUN apt install ros-iron-turtlebot3* -y

RUN echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/opt/ros/iron/share/turtlebot3_gazebo/models'