搞一个preception-aware velocity control

## Requirements
考虑到[官方教程](https://www.youtube.com/watch?v=e3HUKGAWdx0)里面的WSL2限制太多，为了便于部署，PX4在远程server（172.16.15.188）的docker里运行，然后airsim+QGC在本地windows11开发机（172.16.13.104）里运行。

为了测试ROS2的offboard功能，可以把我构建的docker container作为虚拟机，后续验证流程可以参考这个[教程](https://github.com/Jaeyoung-Lim/px4-offboard/blob/master/doc/ROS2_PX4_Offboard_Tutorial.md)

## Install
```sh
docker build -t mypx4_image:v1 .

docker run -itd --privileged -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=$DISPLAY --gpus all --user=user --env=PX4_SIM_HOST_ADDR=172.16.13.104 --network=host --name=mypx4-dev mypx4_image:v1 /bin/bash

docker exec -it --user=user mypx4-dev /bin/bash

sudo chmod 777 -R .
```

## 使用QGC在Airsim里手动控制PX4无人机（Optional）
如果需要手动控制无人机(remote control)，则在QGroundControl里面，必须手动设置通信链接，QGC的自动连接功能在此处不起作用。首先，添加一个14550的UDP监听，并且需要在可选的指定server处添加`172.16.13.104:18570`，并点击连接，随即启动AirSim即可。


## TroubleShooting
### 1. 可以换一台网络好的机器解决docker拉不下来的问题。
```sh
docker save > <image-name>.tar <repository>:<tag>
docker load < <image-name>.tar
```
### 2. 关于"WSL2本地跑PX4+Windows跑AirSim+Windows跑QGC"的连接问题
如果不用docker，而是在WSL本地跑cmake装PX4来调试，连接问题也会很烦。首先解决PX4与airsim的连接问题，需要在windows的powershell里用`ipconfig`来找本机的WSL IPv4 Address，这需要设置到AirSim中`settings.json`的`LocalHostIp`属性，以及上述教程中所有`PX4_SIM_HOST_ADDR`中。之后每次跑PX4以前，甚至需要人为指定环境变量来找windows本机，例如：
```sh
export PX4_SIM_HOST_ADDR=172.18.240.1
```
其次，需要解决PX4与QGC的连接问题，在QGroundControl里面，需要添加一个14550的UDP监听，并且需要在可选的指定server处添加`<wsl-ip>:18570`,其中`wsl-ip`可以在WSL里面输入`ifconfig`查到外部ip地址（针对windows网络的eth0）,每次重启WSL2这个ip都会刷新，形如：`172.18.243.55:18570`，最后的端口也不一定是`18570`，也要注意PX4版本（详见：`PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink`中的`udp_gcs_port_local`）。

### 3. px4编译过程中的自动依赖安装问题
在运行`make px4_sitl_default none_iris`的时候如果遇到警报，可以hit 'u'，避免resolve manually，亲测会省心一点。如果半天卡在`Building for code coverage`，请检查网速是不是太慢。

## 4. 我想修改编译后的UE游戏的窗口等设置
https://blog.csdn.net/qq_33727884/article/details/89487292