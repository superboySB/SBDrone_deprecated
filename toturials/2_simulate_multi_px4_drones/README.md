# Toturials of controling multiple px4 drones
如何在airsim上面部署多个PX4无人机，并实现用QGroundControl（GQC）手动控制


## 使用容器运行多个PX4无人机
编译方式同单无人机教程，启动容器
```sh
docker run -itd --privileged --env=LOCAL_USER_ID="$(id -u)" --env=PX4_SIM_HOST_ADDR=172.16.13.104 -v /home/wangchao/daizipeng/SBDrone:/src:rw -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=:0 --network=host --name=mypx4-n  mypx4_image:v1 /bin/bash
```
其中，`–-env=PX4_SIM_HOST_ADDR==172.16.13.104` 容器添加`PX4_SIM_HOST_ADDR`环境变量，指定远端airsim主机地址；`–-name`后面指定此容器名称。

登录容器
```sh
docker exec -it --user $(id -u) mypx4-n /bin/bash
```
运行多个PX4实例（以4个为例）：
```sh
bash /src/Scripts/run_airsim_sitl.sh 0
bash /src/Scripts/run_airsim_sitl.sh 1
bash /src/Scripts/run_airsim_sitl.sh 2
bash /src/Scripts/run_airsim_sitl.sh 3
```
当前px4处于等待连接服务端的状态，随即，检查和AirSim中setting.json中的同步，再启动AirSim即可。


## 使用QGC在Airsim里手动控制PX4无人机（Optional）
如果需要手动控制无人机(remote control)，则在QGroundControl里面，必须手动设置通信链接，QGC的自动连接功能在此处不起作用。首先，添加一个14550的UDP监听，并且需要在可选的指定server处添加`172.16.13.104:18570`、`172.16.13.104:18571`、`172.16.13.104:18572`、`172.16.13.104:18573`，并点击连接，随即启动AirSim即可。

## TroubleShooting
### 1. 可以换一台网络好的机器解决docker拉不下来的问题。
```sh
docker save > <image-name>.tar <repository>:<tag>
docker load < <image-name>.tar
```
