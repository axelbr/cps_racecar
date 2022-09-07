FROM --platform=linux/arm64 ros:humble
ARG DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
RUN apt update && apt install -y \
  ssh \
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  python3-argcomplete \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  wget \
  ros-${DISTRO}-ament-lint \
  ros-${DISTRO}-launch-testing \
  ros-${DISTRO}-launch-testing-ament-cmake \
  ros-${DISTRO}-launch-testing-ros \
  ros-${DISTRO}-desktop \
  ros-${DISTRO}-libg2o \
  ros-${DISTRO}-xacro \
  ros-${DISTRO}-ros2-control \
  ros-${DISTRO}-ros2-controllers \
  ros-${DISTRO}-urg-node \
  ros-${DISTRO}-joint-state-publisher \
  ros-${DISTRO}-serial-driver \
  ros-${DISTRO}-joy \
  ros-${DISTRO}-joy-teleop
#RUN sed --in-place --expression '$isource "/workspace/install/setup.bash"' /ros_entrypoint.sh
CMD [ "/bin/bash" ]