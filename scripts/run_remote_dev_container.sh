#!/bin/bash

host=${1:-racecar} 
project_dir=${2:-/home/rindt/Projects/cps_racecar} 
container_name=${3:-racecar_dev} 

if [ "$(docker -H ssh://racecar ps -aq -f name=racecar_dev)" ]; then
    if [ "$(docker -H ssh://racecar ps -aq -f name=racecar_dev -f status=exited)" ]; then
        docker -H ssh://racecar start racecar_dev;
    fi
    # run your container
    docker -H ssh://racecar attach racecar_dev;
else
    echo "Container does not exist. Run new instance."
    docker -H ssh://racecar run -it \
	--name racecar_dev \
	-v $project_dir:/workspace \
	-w /workspace \
	--device /dev/sensors/vesc:/dev/sensors/vesc \
	--network host \
	cps_racecar/system:dev bash
fi
