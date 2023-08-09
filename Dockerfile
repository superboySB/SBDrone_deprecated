# 目前使用较为稳定的ros2-foxy (ubuntu-20.04)来做
FROM px4io/px4-dev-ros2-foxy:latest

# 安装系统依赖
RUN apt-get update && \
    apt-get -y install software-properties-common gedit vim tmux net-tools apt-utils git fuse gstreamer1.0-plugins-bad \
    gstreamer1.0-libav gstreamer1.0-gl libqt5gui5 libfuse2 htop

# 下载魔改PX4飞控和offboard样例代码
WORKDIR /home/user
RUN git clone https://github.com/superboySB/PX4-Autopilot.git --recursive && bash PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx && \
    cd PX4-Autopilot && make clean && DONT_RUN=1 make px4_sitl gazebo
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

# 下载魔改AirSim和UE4代码（Ubuntu开发机上应该更加需要，Windows开发机直接用客户端即可完美支持）
WORKDIR /home/user
RUN git clone -b 4.27 git@github.com:EpicGames/UnrealEngine.git && git clone https://github.com/superboySB/AirSim.git

# 暂时需要两种权限的用户使用所有的功能
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc && \
    echo "source /opt/ros/foxy/setup.bash" >> /home/user/.bashrc && pkill -x px4 || true
RUN usermod -aG sudo user && echo 'user ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
WORKDIR /home/user
RUN chmod 777 -R .

# 安装mavros相关的依赖包 (ROS2暂时已经不需要了)
# RUN apt-get install ros-foxy-mavros ros-foxy-mavros-extras && \ 
#     wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
#     bash ./install_geographiclib_datasets.sh

# 在构建镜像时清理 apt 缓存，减小最终镜像的体积（待最终稳定后再加）
# RUN apt-get clean && \
#     rm -rf /var/lib/apt/lists/*
    
