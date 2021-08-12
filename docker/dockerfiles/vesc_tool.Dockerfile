FROM ubuntu:focal
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y build-essential \
  cmake \
  git \
  qt5-default \
  qtpositioning5-dev \
  qtquickcontrols2-5-dev \
  libqt5serialport5-dev \
  qtconnectivity5-dev \
  libqt5gamepad5-dev
RUN git clone https://github.com/vedderb/vesc_tool.git
WORKDIR vesc_tool
RUN qmake -qt=qt5 && make clean && make
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute.
ENV QT_X11_NO_MITSHM 1
ENV QT_DEBUG_PLUGINS 1