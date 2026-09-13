# ROS 2 Humble + Micro-XRCE-DDS-Agent + everything needed to build and run drone-2027.
#
# Build from the repo root:
#   docker build -f docker/Dockerfile -t drone-2027 .
# Run with the repo mounted (add --device /dev/ttyTHS1 for a serial link):
#   docker run -it --rm --net=host -v "$PWD":/home/devuser/drone-2027 drone-2027
FROM ros:humble-ros-base

ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# PX4's DDS client speaks the v2 protocol; v3.x agents are not compatible.
ARG XRCE_AGENT_VERSION=v2.4.2

USER root
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG dialout $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    cmake \
    ffmpeg \
    python3-pip \
    python3-colcon-common-extensions \
    python3-flask \
    python3-flask-cors \
    python3-numpy \
    python3-opencv \
    python3-pytest \
    python3-scipy \
    python3-serial \
    python3-shapely \
    python3-yaml \
    ros-humble-launch-ros \
    ros-humble-sensor-msgs \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Micro XRCE-DDS Agent: the bridge between PX4 and ROS 2 (replaces MAVROS).
RUN git clone --depth 1 -b ${XRCE_AGENT_VERSION} https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce-agent \
    && cmake -S /tmp/xrce-agent -B /tmp/xrce-agent/build \
    && cmake --build /tmp/xrce-agent/build -j"$(nproc)" \
    && cmake --install /tmp/xrce-agent/build \
    && ldconfig /usr/local/lib \
    && rm -rf /tmp/xrce-agent

# MediaMTX for camera streaming.
COPY --from=bluenviron/mediamtx:1 /mediamtx /usr/local/bin/mediamtx

USER $USERNAME
WORKDIR /home/$USERNAME/drone-2027

RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "[ -f /home/$USERNAME/drone-2027/install/setup.bash ] && source /home/$USERNAME/drone-2027/install/setup.bash" >> /home/$USERNAME/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec \"$@\"", "--"]
CMD ["/bin/bash"]
