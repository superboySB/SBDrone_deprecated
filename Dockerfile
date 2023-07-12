# 目前统一使用项目常用的ros1-melodic (ubuntu-18.04)来做
FROM px4io/px4-dev-ros-melodic:latest

# 安装系统依赖
RUN apt-get update && \
    apt-get -y install software-properties-common gedit vim tmux net-tools python-wstool python-catkin-tools && \
    python-rosinstall-generator  

# 将PX4-Autopilot/Tools/setup目录里ubuntu.sh和requirements.txt，拷贝至容器的/tmp目录下
ADD PX4-Autopilot/Tools/setup/ubuntu.sh  PX4-Autopilot/Tools/setup/requirements.txt /tmp/

# 执行ubuntu.sh脚本，安装px4编译相关的依赖包, 不安装nuttx的包
RUN /tmp/ubuntu.sh

# 安装mavros相关的依赖包
RUN apt-get install ros-melodic-mavros ros-melodic-mavros-extras && \ 
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash ./install_geographiclib_datasets.sh

# 在构建镜像时清理 apt 缓存，减小最终镜像的体积
# RUN apt-get clean && \
#     rm -rf /var/lib/apt/lists/*
    