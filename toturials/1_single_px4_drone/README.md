# Toturials of controling a single px4 drone
如何在airsim上面部署一个PX4无人机，并实现用GCS控制

## Requirements
PX4在docker里运行，然后airsim在windows11里用

参考官方教程：https://www.youtube.com/watch?v=e3HUKGAWdx0

## Install
从github克隆PX4代码,通过--recursive参数，将依赖的submodule也一起拉取下来
```sh
cd ~/SBDrone && git clone https://github.com/PX4/PX4-Autopilot.git --recursive && cd PX4-Autopilot && git checkout -b v1.13.3 v1.13.3
```
构建px4镜像
```sh
cd ~/SBDrone && docker build -t mypx4_image:v1 .
```


## 使用容器镜像编译px4
执行以下命令启动容器在后台运行，此容器将本地工程映射至其/src目录。
```sh
docker run -itd --privileged -v /home/qiyuan/SBDrone:/src:rw --network=host --name=mypx4-dev mypx4_image:v1 /bin/bash
```
登录容器
```sh
docker exec -it mypx4-dev /bin/bash
```

**每次有代码更新**，均需要在这个`mypx4-dev`容器终端中执行编译命令,然后`exit`即可
```sh
cd /src/PX4-Autopilot/ && make clean && make px4_sitl_default none_iris
```

## 运行PX4
启动容器
```sh
docker run -itd --privileged --env=LOCAL_USER_ID=1000 \
--env=instance_num=0 \
--env=PX4_SIM_HOST_ADDR=172.18.240.1 \
-v /home/qiyuan/SBDrone:/src:rw \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=:0 \
--network=host \
--name=mypx4-0  \
harbor.tiduyun.com/qujun/px4-dev-ros2-foxy:latest  \
/src/Scripts/run_airsim_sitl.sh
```

## TroubleShooting

### 关于WSL2跑PX4+Windows跑AirSim+Windows跑QGC
如果在WSL本地装PX4来调试，则连接问题会很烦。首先解决PX4与airsim的连接问题，需要在windows的powershell里用`ipconfig`来找本机的WSL IPv4 Address，之后每次跑PX4以前，需要人为指定环境变量来找windows本机，例如：
```sh
export PX4_SIM_HOST_ADDR=172.18.240.1
```
其次，需要解决PX4与QGC的连接问题，在QGroundControl里面，需要添加一个14550的UDP监听，并且需要在可选的指定server处添加`<wsl-ip>:18570`,其中`wsl-ip`可以在WSL里面输入ifconfig查到外部ip地址（针对windows网络的eth0）,每次重启WSL2这个ip都会刷新，形如：`172.18.243.55:18570`

### 关于WSL2跑PX4+Windows跑AirSim+Android跑QGC