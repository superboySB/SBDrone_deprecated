# 目前统一使用项目常用的ros1-melodic (ubuntu-18.04)来做
FROM px4io/px4-dev-ros-melodic:latest

RUN apt-get update && apt-get -y install tmux net-tools

# 将PX4-Autopilot/Tools/setup目录里ubuntu.sh和requirements.txt，拷贝至容器的/tmp目录下
ADD PX4-Autopilot/Tools/setup/ubuntu.sh  PX4-Autopilot/Tools/setup/requirements.txt /tmp/

# 执行ubuntu.sh脚本，安装px4编译相关的依赖包, 不安装nuttx和模拟器的包
RUN /tmp/ubuntu.sh --no-nuttx --no-sim-tools
    