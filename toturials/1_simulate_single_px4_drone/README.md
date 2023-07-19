# Toturials of controling a single px4 drone
如何在airsim上面部署一个PX4无人机，并实现用QGroundControl（GQC）手动控制

## Requirements
考虑到[官方教程](https://www.youtube.com/watch?v=e3HUKGAWdx0)里面的WSL2限制太多，为了便于部署，PX4在远程server（172.16.15.188）的docker里运行，然后airsim+QGC在本地windows11开发机（172.16.13.104）里运行。

## Install
从github克隆PX4代码,通过--recursive参数，将依赖的submodule也一起拉取下来
```sh
cd ~/SBDrone && git clone https://github.com/PX4/PX4-Autopilot.git --recursive && cd PX4-Autopilot && git checkout -b v1.13.3 v1.13.3
```
构建px4镜像
```sh
cd ~/SBDrone && docker build -t mypx4_image:v1 .
```

## 使用容器编译PX4源码
执行以下命令启动容器在server后台运行，此容器将本地工程映射至其/src目录。
```sh
docker run -itd --privileged -v /home/wangchao/daizipeng/SBDrone:/src:rw -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=:0 --gpus all -env=LOCAL_USER_ID="$(id -u)" --network=host --name=mypx4-dev mypx4_image:v1 /bin/bash
```
登录容器
```sh
docker exec -it --user $(id -u) mypx4-dev /bin/bash
```

**每次有代码更新**，均需要在这个`mypx4-dev`容器终端中执行编译命令,然后`exit`即可
```sh
cd /src/PX4-Autopilot/ && make clean && make px4_sitl_default none_iris
```

## 使用容器运行PX4无人机
在server中启动容器，并以下面命令为例，打开一个无人机的instance：
```sh
docker run -itd --privileged --env=LOCAL_USER_ID="$(id -u)" --env=PX4_SIM_HOST_ADDR=172.16.13.104 -v /home/wangchao/daizipeng/SBDrone:/src:rw -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=:0 --network=host --gpus all --name=mypx4-0  mypx4_image:v1 bash /src/Scripts/run_airsim_sitl.sh
```
其中，`–-env=PX4_SIM_HOST_ADDR=172.16.13.104` 容器添加`PX4_SIM_HOST_ADDR`环境变量，指定远端airsim主机地址；`–-name`后面指定此容器名称；`/src/Scripts/run_airsim_sitl.sh`表示容器执行此命令启动。容器启动后，使用`docker logs -f mypx4-0`将出现日志，当前px4处于等待连接服务端的状态，随即启动AirSim即可。


## 使用QGC在Airsim里手动控制PX4无人机（Optional）
如果需要手动控制无人机(remote control)，则在QGroundControl里面，必须手动设置通信链接，QGC的自动连接功能在此处不起作用。首先，添加一个14550的UDP监听，并且需要在可选的指定server处添加`172.16.13.104:18570`，并点击连接，随即启动AirSim即可。



## TroubleShooting
### 1. 可以换一台网络好的机器解决docker拉不下来的问题。
```sh
docker save > <image-name>.tar <repository>:<tag>
docker load < <image-name>.tar
```

### 2. 关于WSL2本地跑PX4+Windows跑AirSim+Windows跑QGC的连接问题
如果不用docker，而是在WSL本地跑cmake装PX4来调试，连接问题也会很烦。首先解决PX4与airsim的连接问题，需要在windows的powershell里用`ipconfig`来找本机的WSL IPv4 Address，这需要设置到AirSim中`settings.json`的`LocalHostIp`属性，以及上述教程中所有`PX4_SIM_HOST_ADDR`中。之后每次跑PX4以前，甚至需要人为指定环境变量来找windows本机，例如：
```sh
export PX4_SIM_HOST_ADDR=172.18.240.1
```
其次，需要解决PX4与QGC的连接问题，在QGroundControl里面，需要添加一个14550的UDP监听，并且需要在可选的指定server处添加`<wsl-ip>:18570`,其中`wsl-ip`可以在WSL里面输入`ifconfig`查到外部ip地址（针对windows网络的eth0）,每次重启WSL2这个ip都会刷新，形如：`172.18.243.55:18570`，最后的端口也不一定是`18570`，也要注意PX4版本（详见：`PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink`中的`udp_gcs_port_local`）。

### 3. px4编译过程中的自动依赖安装问题
在运行`make px4_sitl_default none_iris`的时候如果遇到警报，可以hit 'u'，避免resolve manually，亲测会省心一点。如果半天卡在`Building for code coverage`，请检查网速是不是太慢。

### 4. 在docker里面实例化PX4 instance连接不上QGC的问题（依然连不上）
针对`INFO [mavlink] MAVLink only on localhost (set param MAV_{i}_BROADCAST = 1 to enable network)`的问题，我理解是你想在docker里面实例化PX4 instance，实现与外部QGC的连接，如果可以正常连接则无视这个问题。否则，这里需要在PX4的build目录下，在`/etc/init.d-posix`找到`px4-rc.mavlink`文件，并且将udp-gcs的mavlink实例化代码添加`-p`标志符，即改为`mavlink start -x -u $udp_gcs_port_local -r 4000000 -f -p`，然后在同目录下的`px4-rc.params`文件添加`param set-default MAV_0_BROADCAST 1`。最后，再重新使用mypx4-dev来编译代码。

### 5. 关于WSL2跑PX4+Windows跑AirSim+Android跑QGC
TODO

