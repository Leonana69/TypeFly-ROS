FROM ros:humble-ros-base-jammy

RUN apt update && apt install ros-humble-desktop-full -y
RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc

RUN apt install ros-humble-slam-toolbox -y
RUN apt install ros-humble-navigation2 -y
RUN apt install ros-humble-nav2-bringup -y
RUN apt install ros-humble-turtlebot3* -y

RUN apt update && apt install nano wget xdg-utils -y

# Install VSCode
RUN wget -O code.deb https://vscode.download.prss.microsoft.com/dbazure/download/stable/e170252f762678dec6ca2cc69aba1570769a5d39/code_1.88.1-1712771838_amd64.deb
RUN dpkg -i code.deb && rm code.deb

# Install sublime-text
# RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F57D4F59BD3DF454
# RUN wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/sublimehq-archive.gpg > /dev/null
# RUN echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
# RUN apt update
# RUN apt install -y sublime-text

RUN echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
RUN echo 'alias code-open="code --no-sandbox --user-data-dir /tmp"' >> ~/.bashrc

RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

COPY ./pod_sensing /root/ros2_ws/src/pod_sensing

CMD ["bash"]