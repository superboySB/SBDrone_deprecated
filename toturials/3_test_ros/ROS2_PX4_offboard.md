按照我的镜像来构建环境，后续验证流程可以参考这个[教程](https://github.com/Jaeyoung-Lim/px4-offboard/blob/master/doc/ROS2_PX4_Offboard_Tutorial.md)
```sh
docker run -itd --privileged -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=$DISPLAY --gpus all --env=LOCAL_USER_ID="$(id -u)" --network=host --name=mypx4-dev mypx4_image:v1 /bin/bash

docker exec -it --user $(id -u) mypx4-dev /bin/bash
```