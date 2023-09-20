# 拉取镜像前明确相应配置
ARG DEBIAN_FRONTEND=noninteractive
ARG BASE_DIST=ubuntu20.04
ARG CUDA_VERSION=11.4.2
ARG ISAACSIM_VERSION=2022.2.1

# https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim
FROM nvcr.io/nvidia/isaac-sim:${ISAACSIM_VERSION} as isaac-sim

# https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cudagl
FROM nvidia/cudagl:${CUDA_VERSION}-base-${BASE_DIST}



FROM px4io/px4-dev-ros2-foxy:latest


# 安装系统依赖
RUN apt-get update && \
    apt-get -y --no-install-recommends install software-properties-common gedit vim tmux net-tools apt-utils git fuse gstreamer1.0-plugins-bad \
    gstreamer1.0-libav gstreamer1.0-gl libqt5gui5 libfuse2 htop libxcursor-dev libxrandr-dev libxinerama-dev libxi-dev mesa-common-dev make zip \
    unzip vulkan-utils mesa-vulkan-drivers pigz libegl1 git-lfs gcc-8 g++-8 python3-tk

# Force gcc 8 to avoid CUDA 10 build issues on newer base OS
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8

# WAR for eglReleaseThread shutdown crash in libEGL_mesa.so.0 (ensure it's never detected/loaded)
# Can't remove package libegl-mesa0 directly (because of libegl1 which we need)
RUN rm /usr/lib/x86_64-linux-gnu/libEGL_mesa.so.0 /usr/lib/x86_64-linux-gnu/libEGL_mesa.so.0.0.0 /usr/share/glvnd/egl_vendor.d/50_mesa.json
COPY docker/nvidia_icd.json /usr/share/vulkan/icd.d/nvidia_icd.json
COPY docker/10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# 下载魔改PX4飞控和offboard样例代码
WORKDIR /home/user
RUN git clone https://github.com/superboySB/PX4-Autopilot.git --recursive && bash PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
RUN cd PX4-Autopilot && make clean && DONT_RUN=1 make px4_sitl_default gazebo-classic
WORKDIR /home/user
RUN git clone https://github.com/Jaeyoung-Lim/px4-offboard.git && cd px4-offboard && colcon build && cd ..
RUN wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage && chmod +x ./QGroundControl.AppImage

# 安装px4_msgs和micro_ros_agent
RUN . /opt/ros/$ROS_DISTRO/setup.sh && mkdir -p /home/user/px4_ros_com_ws/src && cd /home/user/px4_ros_com_ws/src && \
    git clone https://github.com/PX4/px4_msgs.git && cd .. && colcon build
WORKDIR /home/user
RUN . /opt/ros/$ROS_DISTRO/setup.sh && mkdir microros_ws && cd microros_ws && \
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    rosdep update && rosdep install --from-paths src --ignore-src -y && colcon build && . install/local_setup.sh && \
    ros2 run micro_ros_setup create_agent_ws.sh && ros2 run micro_ros_setup build_agent.sh


# [硬盘空间若不足1T一定慎选] 下载UE4和魔改AirSim的代码，并编译默认版本（Ubuntu开发机上跑仿真需要，但建议还是用Windows开发机跑仿真，客户端可完美支持编译源码素材）
# [Windows可参考教程：https://www.zhihu.com/column/multiUAV]
# WORKDIR /home/user  
# RUN git clone --single-branch --branch 4.27 --depth 1 https://<personal-github-token>@github.com/EpicGames/UnrealEngine.git
# RUN cd UnrealEngine && bash ./Setup.sh && bash ./GenerateProjectFiles.sh && make
# WORKDIR /home/user
# RUN git clone https://github.com/superboySB/AirSim.git
# RUN cd AirSim && bash ./setup.sh && bash ./build.sh

# [ROS2暂时已经不需要了] 安装mavros相关的依赖包
# RUN apt-get install ros-foxy-mavros ros-foxy-mavros-extras && \ 
#     wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
#     bash ./install_geographiclib_datasets.sh

# [暂时先安装它的爸爸] 安装isaacgym
# [注意] 涉及License问题，需要先去获取IsaacGym_Preview_4_Package，并且将里面的isaacgym拖到项目目录中
# COPY isaacgym /home/user/isaacgym/
# ENV PATH="/home/user/.local/bin:$PATH"
# RUN cd /home/user/isaacgym/python && pip3 install --upgrade pip && pip3 install -e .
# ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all

# 安装IsaacGymEnvs
# WORKDIR /home/user
# RUN git clone https://github.com/NVIDIA-Omniverse/IsaacGymEnvs.git && cd IsaacGymEnvs && pip install -e .

# 安装pytorch3D
WORKDIR /home/user
RUN git clone https://github.com/facebookresearch/pytorch3d.git && cd pytorch3d && pip3 install -e .

# 安装sample factory
WORKDIR /home/user
RUN pip3 install opencv-python opencv-contrib-python msgpack-rpc-python sympy tkinter
RUN git clone https://github.com/superboySB/sample-factory.git && cd sample-factory && pip3 install -e .

# 暂时需要两种权限的用户使用所有的功能
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc && \
    echo "source /opt/ros/foxy/setup.bash" >> /home/user/.bashrc && pkill -x px4 || true
RUN usermod -aG sudo user && echo 'user ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
WORKDIR /home/user
RUN chmod 777 -R .


# [待最终稳定后再加] 在构建镜像时清理 apt 缓存，减小最终镜像的体积
# RUN apt-get clean && \
#     rm -rf /var/lib/apt/lists/*
    
