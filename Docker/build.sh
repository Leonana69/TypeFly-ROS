# remove old container and image
docker stop -t 0 typefly-ros
docker image rm -f typefly-ros:0.1
docker rm -f typefly-ros &>/dev/null

# build
docker build -t typefly-ros:0.1 .