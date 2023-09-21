# Notes for re-implementing paper "PRL4AirSim"

尝试带着PX4做强化学习，从一个无人机开始

## Requirements

集成必要的环境
```sh
docker build -t mypx4_image:v1 .

docker run -itd --privileged -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=$DISPLAY --gpus all --user=user --env=PX4_SIM_HOST_ADDR=172.16.13.104 --network=host --name=mypx4-dev mypx4_image:v1 /bin/bash

docker exec -it --user=user mypx4-dev /bin/bash

git clone https://github.com/superboySB/SBDrone && cd cd SBDrone && pip install -r requirements.txt
```

```sh
bash /home/user/PX4-Autopilot/Tools/simulation/sitl_multiple_run.sh 1
```

/home/user/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 0 -d /home/user/PX4-Autopilot/build/px4_sitl_default/etc >out.log 2>err.log &


## TroubleShooting
### 1. 可以换一台网络好的机器解决docker拉不下来的问题。
```sh
docker save > <image-name>.tar <repository>:<tag>
docker load < <image-name>.tar
```
### 2. 修改AirSim屏幕分辨率
https://blog.csdn.net/qq_33727884/article/details/89487292


### 3. 建飞老师打的命令
```sh
mavlink status
listener manual_control_setpoint -r 10
listener input_rc
```