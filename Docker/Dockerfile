FROM ros:iron-ros-base-jammy

RUN apt update && apt install nano -y
RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc

RUN apt install ros-iron-slam-toolbox -y
RUN apt install ros-iron-navigation2 -y
RUN apt install ros-iron-nav2-bringup -y
RUN apt install ros-iron-turtlebot3* -y

# Install sublime-text
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F57D4F59BD3DF454
RUN wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/sublimehq-archive.gpg > /dev/null
RUN echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
RUN apt update
RUN apt install -y sublime-text

RUN apt install ros-iron-desktop-full -y

RUN echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
WORKDIR /root/catkin_ws
CMD ["bash"]