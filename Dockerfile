#syntax=docker/dockerfile:1.4.0
ARG CL_VERSION=v7.1.1
ARG NETIF_VERSION=v1.4.1
ARG ROS2_VERSION=humble
FROM ghcr.io/aica-technology/control-libraries:${CL_VERSION} as cl
FROM ghcr.io/aica-technology/network-interfaces:${NETIF_VERSION} as netif
FROM ghcr.io/aica-technology/ros2-ws:${ROS2_VERSION} as base
# install dependencies
COPY --from=cl / /
COPY --from=netif / /
RUN sudo apt-get update && sudo apt-get install -y \
  libxinerama1 \
  libxcursor-dev \
  && sudo ldconfig \
  && sudo rm -rf /var/lib/apt/lists/*

# install mujoco  https://github.com/google-deepmind/mujoco/releases/download/2.3.7/mujoco-2.3.7-linux-x86_64.tar.gz
RUN mkdir -p ${HOME}/mujoco \
    && wget https://github.com/google-deepmind/mujoco/releases/download/2.3.7/mujoco-2.3.7-linux-x86_64.tar.gz -O mujoco.tar.gz \
    && tar -xf mujoco.tar.gz -C ${HOME}/mujoco \
    && rm mujoco.tar.gz

RUN pip install mujoco
