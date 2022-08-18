#!/bin/bash
sudo xhost +local:docker
docker run --rm -it --privileged -v $(pwd):/cps_racecar --volume=/tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev -e DISPLAY=$DISPLAY --device=/dev/input/js0 --network host vesc_tool bash

