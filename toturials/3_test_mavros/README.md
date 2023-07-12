# Toturials of mavros and px4
如何在airsim上面用MAVROS给PX4无人机发送话题控制


## 从Source安装mavros
源码编译方式同单无人机教程，需要先在“编译用容器”里编译，然后再启动“运行用容器”如下
```sh
docker run -itd --privileged --env=LOCAL_USER_ID="$(id -u)" --env=PX4_SIM_HOST_ADDR=172.16.13.104 -v /home/wangchao/daizipeng/SBDrone:/src:rw -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=:0 --network=host --name=mypx4-0  mypx4_image:v1 /bin/bash
```
其中，`–-env=PX4_SIM_HOST_ADDR=172.18.240.1` 容器添加`PX4_SIM_HOST_ADDR`环境变量，指定远端airsim主机地址；`–-name`后面指定此容器名称。


## 逐步开启mavros服务
在windows设备中，先检查AirSim中setting.json，启动AirSim的某一个map，进入等待服务状态。然后，登录容器
```sh
docker exec -it --user $(id -u) mypx4-0 /bin/bash
```
打开一个窗口，运行2个PX4实例，需要观察到Airsim中有QGC（GPS lock）相关的提示才算成功：
```sh
bash /src/Scripts/run_airsim_sitl.sh 0
bash /src/Scripts/run_airsim_sitl.sh 1
```
注意每次使用ros相关命令时需要输入
```sh
source /opt/ros/melodic/setup.bash
```
打开一个窗口，运行mavros服务，其中第一个端口指定本地主机（127.0.0.1）上的接收端口号（udp_onboard_payload_port_remote），第二个端口指定飞行控制器上的发送端口号（udp_onboard_payload_port_local）。这些可以在上一个窗口的运行日志中，在mavlink的onboard udp port对应上。
```sh
roslaunch mavros px4.launch fcu_url:=udp://:14030@127.0.0.1:14280
roslaunch mavros px4.launch fcu_url:=udp://:14031@127.0.0.1:14281
```

## 使用mavros话题通信在Airsim里手动控制PX4无人机（有点受限于版本V1.12.1）
参考[教程](https://www.youtube.com/watch?v=ZonkdMcwXH4),打开一个窗口，基于mavros发送服务调用指令给px4，实现对无人机的控制，这里给出依次玩耍这些指令的结果：
```sh
# 发起起飞指令，此时不能起飞
rosservice call /mavros/cmd/takeoff "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 0.0}"

# 解锁无人机，此时可以起飞
rosservice call /mavros/cmd/arming "value: true"

# 无人机起飞
rosservice call /mavros/cmd/arming "value: true"

# 无人机降落
rosservice call /mavros/cmd/land "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 0.0}"
```
也可以基于mavros发送话题给px4，以下是开一个窗口跑position controller：
```sh
# 发送position controller的话题指令
rostopic pub /mavros/setpoint_position/local geometry_msgs/PoseStamped "header:
    seq: 0
    stamp:
        secs: 0
        nsecs: 0
    frame_id: ''
pose:
    position:
        x: 1.0
        y: 0.0
        z: 2.0
    orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 0.0" -r 20
```
然后再换个窗口设置飞行模式
```sh
# 该服务的目的是让飞行控制器（例如PX4）切换到特定的飞行模式，这里使用的是'OFFBOARD'模式，该模式允许飞行控制器接受来自外部计算机的指令控制飞行。
rosservice call /mavros/set_mode "base mode: 0
custom_mode: 'OFFBOARD'"

# 解锁无人机，执行指令
rosservice call /mavros/cmd/arming "value: true"

# 可以继续发送其它position controller的话题指令
```
以下是velocity controller的画圈demo：
```sh
rostopic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/TwistStamped "header
    seq: 0
    stamp:
        secs: 0
        nsecs: 0
    frame_id: ''
twist:
    linear:
        x: 1.0
        y: 0.0
        z: 0.0
    angular:
        x: 0.0
        y: 0.0
        z: 1.0" -r 20
```


## 运行开发机上的Airsim
如果需要手动控制无人机(remote control)，则在QGroundControl里面，必须手动设置通信链接，QGC的自动连接功能在此处不起作用。首先，添加一个14550的UDP监听，并且需要在可选的指定server处添加`172.16.13.104:18570`、`172.16.13.104:18571`、`172.16.13.104:18572`、`172.16.13.104:18573`，并点击连接，随即启动AirSim即可。


## TroubleShooting
### 1. 可以换一台网络好的机器解决docker拉不下来的问题。
```sh
docker save > <image-name>.tar <repository>:<tag>
docker load < <image-name>.tar
```
