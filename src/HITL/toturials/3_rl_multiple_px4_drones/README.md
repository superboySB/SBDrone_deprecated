# Notes for re-implementing paper "PRL4AirSim"

复现论文PRL4AirSim.

## Requirements
这个原论文自带的binary编译自某个windows editor项目，但开源只提供了linux版本，所以应该整个项目暂时都是将一台linux的机器作为host machine

## Install
```sh
docker build -t mypx4_image:v1 .

docker run -itd --privileged -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=$DISPLAY --gpus all --user=user --env=PX4_SIM_HOST_ADDR=172.16.13.104 --network=host --name=mypx4-dev mypx4_image:v1 /bin/bash

docker exec -it --user=user mypx4-dev /bin/bash

bash PX4-Autopilot/Tools/simulation/sitl_multiple_run.sh 2


cd PRL4AirSim && pip install -r requirements.txt
```





## TroubleShooting
### 1. 可以换一台网络好的机器解决docker拉不下来的问题。
```sh
docker save > <image-name>.tar <repository>:<tag>
docker load < <image-name>.tar
```
### 2. 如果使用原版AirSim，遇到UE4.27跑不了Blocks实例的问题
