#!/bin/bash

CONTAINER=""

while [ "$1" != "" ]; do
    case "$1" in
        -d | --docker )  CONTAINER="$2";  shift;;
    esac
    shift
done

IMAGE_NAME=${CONTAINER}

NVIDIA_FLAG=""
if [[ $IMAGE_NAME = *"nvidia"* ]]; then
  NVIDIA_FLAG="--runtime=nvidia"
fi

xhost +local:docker
docker run -it \
    --privileged --rm \
    "--ipc=host" \
    "--cap-add=IPC_LOCK" \
    "--cap-add=sys_nice" \
    "--network=host" \
    --env="DISPLAY"  \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/$USER/create_ws/src:/create_ws/src" \
    -e ROS_HOSTNAME=localhost \
    -e ROS_MASTER_URI=http://localhost:11311 \
    $NVIDIA_FLAG \
    create_nvidia
