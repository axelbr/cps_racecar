#!/bin/bash
sudo xhost +local:docker
docker run --rm -it --privileged --volume=/tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY --network host vesc_tool ./vesc_tool_3.00

