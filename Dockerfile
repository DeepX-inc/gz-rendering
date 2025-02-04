
# Adds DeepX Gazebo Rendering image.
#
# Docker commands:
# - To build the image:
#   docker build -t deepx-gz-rendering .
# - To publish the image:
#   docker push ghcr.io/deepx-inc/deepx-gz-rendering

FROM ghcr.io/deepx-inc/base_images:humble

# ---BUILD TOOLS---
RUN apt-get -qq update \
    && apt-get -qq install -y --no-install-recommends \
    cmake=3.22.* \
    ffmpeg=7:4.4.* \
    freeglut3-dev=2.8.* \
    git=1:2.34.* \
    gnupg=2.2.* \
    libfreeimage-dev=3.18.* \
    libglew-dev=2.2.* \
    libogre-next-dev=2.2.* \
    libxi-dev=2:1.* \
    libxmu-dev=2:1.* \
    ninja-build=1.10.* \
    pkg-config=0.29.* \
    software-properties-common=0.99.22.* \
    lsb-release=11.1.* \
    wget=1.21.* \
    curl=7.81.*

# ---STANDARD GAZEBO INSTALL---
RUN apt-get -qq update && \
    sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-prerelease `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-prerelease.list' && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    add-apt-repository ppa:kisak/kisak-mesa && \
    curl -sSL http://get.gazebosim.org | sh && \
    apt-get -qq update && \
    apt-get -qq upgrade -y --no-install-recommends \
    libgz-cmake3-dev=3.* \
    libgz-common5-dev=5.* \
    libgz-fuel-tools8-dev=8.* \
    libgz-gui7-dev=7.* \
    libgz-launch6-dev=6.* \
    libgz-math7-dev=7.* \
    libgz-msgs9-dev=9.* \
    libgz-physics6-dev=6.* \
    libgz-plugin2-dev=2.* \
    libgz-sim7-dev=7.* \
    libgz-tools2-dev=2.* \
    libgz-transport12-dev=12.* \
    libgz-utils2-dev=2.* \
    libsdformat13=13.* \
    && apt-get -qq -y autoclean \
    && apt-get -qq -y autoremove \
    && rm -rf /var/lib/apt/lists/*

WORKDIR "/root/gazebo"

COPY . gz-rendering/

RUN mkdir -p gz-rendering/build
