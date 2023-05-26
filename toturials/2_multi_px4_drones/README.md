# Toturials of controling multiple px4 drones

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