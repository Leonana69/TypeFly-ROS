# create a new container
docker run -td --privileged --net=host --ipc=host \
    --name="typefly-ros" \
    -e "DISPLAY=$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    typefly-ros:0.1