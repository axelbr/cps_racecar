FROM ros:melodic-robot

# install ros package
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-driver-base \
      ros-${ROS_DISTRO}-urg-node \
      libyaml-cpp-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/src
RUN git clone https://github.com/axelbr/f1tenth_system.git && \
    find . -name “*.py” -exec chmod +x {} \;
WORKDIR /workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make"
RUN sed --in-place --expression \
      '$isource "/workspace/devel/setup.bash"' \
      /ros_entrypoint.sh
