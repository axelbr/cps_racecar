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
  libqt5gamepad5-dev \
  wget
RUN wget https://github.com/rpasichnyk/vesc_tool/archive/refs/tags/v3.0.0.zip && unzip v3.0.0.zip
WORKDIR vesc_tool-3.0.0
RUN qmake -qt=qt5 && make clean && make
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute.
ENV QT_X11_NO_MITSHM 1
ENV QT_DEBUG_PLUGINS 1