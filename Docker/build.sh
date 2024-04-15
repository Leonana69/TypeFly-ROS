# UI permisions
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

xhost +local:docker

docker stop -t 0 typefly-ros
docker image rm -f typefly-ros:0.1
docker rm -f typefly-ros &>/dev/null

# build
docker build -t typefly-ros:0.1 .
docker run -td --privileged --net=host --ipc=host \
    --name="typefly-ros" \
    -e "DISPLAY=$DISPLAY" \
    -e "QT_X11_NO_MITSHM=1" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e "XAUTHORITY=$XAUTH" \
    --cap-add=SYS_PTRACE \
    typefly-ros:0.1