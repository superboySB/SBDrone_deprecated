# https://hub.docker.com/_/ubuntu/tags
# FROM ubuntu:focal as builder
# RUN apt-get update
# RUN apt-get install -y curl
# RUN apt-get install -y --no-install-recommends gcc libc-dev
# RUN curl -o /usr/local/bin/su-exec.c https://raw.githubusercontent.com/ncopa/su-exec/master/su-exec.c
# RUN gcc -Wall /usr/local/bin/su-exec.c -o/usr/local/bin/su-exec
# RUN chown root:root /usr/local/bin/su-exec
# RUN chmod 0755 /usr/local/bin/su-exec

ARG DEBIAN_FRONTEND=noninteractive
ARG BASE_DIST=ubuntu20.04
ARG CUDA_VERSION=11.4.2
ARG ISAACSIM_VERSION=2022.2.1

# https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim
FROM nvcr.io/nvidia/isaac-sim:${ISAACSIM_VERSION} as isaac-sim

# https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cudagl
FROM nvidia/cudagl:${CUDA_VERSION}-base-${BASE_DIST}

# Please contact with me if you have problems
LABEL maintainer="Zipeng Dai <daizipeng@bit.edu.cn>"

# Setup the required capabilities for the container runtime 
ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8
ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all
ENV OMNI_SERVER http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2022.2.1
ENV MIN_DRIVER_VERSION 525.60.11
ENV ROS_DISTRO=foxy

# ROS2-foxy
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -yq --no-install-recommends keyboard-configuration language-pack-en wget curl git build-essential ca-certificates \
    tzdata tmux gnupg2 vim sudo lsb-release locales bash-completion iproute2 iputils-ping net-tools dnsutils htop make zip unzip cmake dos2unix
RUN locale-gen en_US.UTF-8
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-desktop python3-rosinstall-generator python3-wstool python3-argcomplete \
    python3-colcon-common-extensions python3-rosdep python3-colcon-mixin python3-vcstool ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher-gui
RUN rosdep init && rosdep update


# Install px4 autopilot and offboard api
WORKDIR /workspace
RUN git clone https://github.com/superboySB/PX4-Autopilot.git --recursive && bash PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
RUN cd PX4-Autopilot && make clean && DONT_RUN=1 make px4_sitl_default gazebo-classic
WORKDIR /workspace
RUN git clone https://github.com/Jaeyoung-Lim/px4-offboard.git && cd px4-offboard && colcon build && cd ..
RUN wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage && chmod +x ./QGroundControl.AppImage

# Install px4_msgs and micro_ros_agent
RUN . /opt/ros/$ROS_DISTRO/setup.sh && mkdir -p /workspace/px4_ros_com_ws/src && cd /workspace/px4_ros_com_ws/src && \
    git clone https://github.com/PX4/px4_msgs.git && cd .. && colcon build
WORKDIR /workspace
RUN . /opt/ros/$ROS_DISTRO/setup.sh && mkdir microros_ws && cd microros_ws && \
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    rosdep update && rosdep install --from-paths src --ignore-src -y && colcon build && . install/local_setup.sh && \
    ros2 run micro_ros_setup create_agent_ws.sh && ros2 run micro_ros_setup build_agent.sh

# Issac-Sim Dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libatomic1 libegl1 libglu1-mesa libgomp1 libsm6 libxi6 libxrandr2 \
    libxt6 libfreetype-dev libfontconfig1 openssl libssl1.1 wget vulkan-utils \
    && apt-get -y autoremove \
    && apt-get clean autoclean \
    && rm -rf /var/lib/apt/lists/*

# [Optional] for ubuntu-desktop
WORKDIR /workspace
RUN wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Download the Vulkan SDK and extract the headers, loaders, layers and binary utilities
ARG VULKAN_SDK_VERSION=1.3.224.1 
RUN wget -q --show-progress \
    --progress=bar:force:noscroll \
    https://sdk.lunarg.com/sdk/download/${VULKAN_SDK_VERSION}/linux/vulkansdk-linux-x86_64-${VULKAN_SDK_VERSION}.tar.gz \
    -O /tmp/vulkansdk-linux-x86_64-${VULKAN_SDK_VERSION}.tar.gz \ 
    && echo "Installing Vulkan SDK ${VULKAN_SDK_VERSION}" \
    && mkdir -p /opt/vulkan \
    && tar -xf /tmp/vulkansdk-linux-x86_64-${VULKAN_SDK_VERSION}.tar.gz -C /opt/vulkan \
    && mkdir -p /usr/local/include/ && cp -ra /opt/vulkan/${VULKAN_SDK_VERSION}/x86_64/include/* /usr/local/include/ \
    && mkdir -p /usr/local/lib && cp -ra /opt/vulkan/${VULKAN_SDK_VERSION}/x86_64/lib/* /usr/local/lib/ \
    && cp -a /opt/vulkan/${VULKAN_SDK_VERSION}/x86_64/lib/libVkLayer_*.so /usr/local/lib \
    && mkdir -p /usr/local/share/vulkan/explicit_layer.d \
    && cp /opt/vulkan/${VULKAN_SDK_VERSION}/x86_64/etc/vulkan/explicit_layer.d/VkLayer_*.json /usr/local/share/vulkan/explicit_layer.d \
    && mkdir -p /usr/local/share/vulkan/registry \
    && cp -a /opt/vulkan/${VULKAN_SDK_VERSION}/x86_64/share/vulkan/registry/* /usr/local/share/vulkan/registry \
    && cp -a /opt/vulkan/${VULKAN_SDK_VERSION}/x86_64/bin/* /usr/local/bin \
    && ldconfig \
    && rm /tmp/vulkansdk-linux-x86_64-${VULKAN_SDK_VERSION}.tar.gz && rm -rf /opt/vulkan

# Open ports for live streaming
EXPOSE 47995-48012/udp \
       47995-48012/tcp \
       49000-49007/udp \
       49000-49007/tcp \
       49100/tcp \
       8011/tcp \
       8012/tcp \
       8211/tcp \
       8899/tcp \
       8891/tcp

# Copy Isaac Sim files
COPY --from=isaac-sim /isaac-sim /isaac-sim
RUN mkdir -p /root/.nvidia-omniverse/config
COPY --from=isaac-sim /root/.nvidia-omniverse/config /root/.nvidia-omniverse/config
COPY --from=isaac-sim /etc/vulkan/icd.d/nvidia_icd.json /etc/vulkan/icd.d/nvidia_icd.json
COPY --from=isaac-sim /etc/vulkan/icd.d/nvidia_icd.json /etc/vulkan/implicit_layer.d/nvidia_layers.json


# OmniIssacGym
WORKDIR /workspace
RUN git clone https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs && cd OmniIsaacGymEnvs \
    && /isaac-sim/python.sh -m pip install -U pip && /isaac-sim/python.sh -m pip install -e . \
    && /isaac-sim/python.sh -m pip install urllib3==1.26

# Orbit
WORKDIR /workspace
RUN git clone https://github.com/superboySB/Orbit
# [Not Stable Now, Not Install Now]
# ENV ISAACSIM_PATH=/isaac-sim
# ENV TERM=xterm
# RUN git clone https://github.com/superboySB/Orbit && chmod 777 -R . && cd Orbit \
#     && ln -s ${ISAACSIM_PATH} _isaac_sim && dos2unix ./orbit.sh \
#     && ./orbit.sh -p -m pip install setuptools==65.5.1 wheel==0.38.4 \
#     && ./orbit.sh --install && ./orbit.sh --extra

# [Optional] if your network is not very well and you just want to use my code directly
WORKDIR /workspace
RUN git clone https://github.com/superboySB/SBDrone.git

# Add symlink
WORKDIR /isaac-sim
RUN ln -s exts/omni.isaac.examples/omni/isaac/examples extension_examples
COPY fastdds.xml ~/.ros/fastdds.xml
COPY 10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json
RUN unset LD_LIBRARY_PATH && echo "export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml" > ~/.bashrc


RUN echo "Finished! Enjoy!"
WORKDIR /workspace
RUN chmod 777 -R .