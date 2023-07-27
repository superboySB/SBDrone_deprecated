# Notes for re-implementing paper "PRL4AirSim"

复现论文PRL4AirSim.

## Requirements
这个原论文自带的binary编译自某个windows editor项目，但开源只提供了linux版本，所以应该整个项目暂时都是将一台linux的机器作为host machine

## Install
注册Epic Games用户，然后编译UE4到本地的某个地方，这个项目有点大(20G+)，需要注意git本身的网络配置、剩余空间
```sh
git config --global http.postBuffer 524288000 && git clone -b 4.27 https://github.com/EpicGames/UnrealEngine && cd UnrealEngine && bash ./Setup.sh && bash ./GenerateProjectFiles.sh && make
```
如果支持图形界面，可如下检验UE是否安装成功
```sh
cd /home/qiyuan/UnrealEngine/Engine/Binaries/Linux && ./UE4Editor
```
从github克隆我根据论文官方repo魔改的AirSim代码，然后编译：
```sh

git clone https://github.com/superboySB/AirSim && cd AirSim && bash ./setup.sh && bash ./build.sh
```
安装Python依赖
```sh
cd PRL4AirSim && pip install -r requirements.txt
```





从github克隆PRL4AirSim原本代码，构建原版docker镜像，
```sh
cd ~/SBDrone && git clone https://github.com/SaundersJE97/PRL4AirSim && cd PRL4Airsim/docker && python build_airsim_image.py --base_image=nvidia/cudagl:10.0-devel-ubuntu18.04 --target_image=airsim_binary:10.0-devel-ubuntu18.04
```
检查镜像是否安装成功
```sh
docker images | grep airsim
```


## TroubleShooting
### 1. 如果使用原版AirSim，遇到UE4.27跑不了Blocks实例的问题
这儿有个[issue](https://github.com/microsoft/AirSim/issues/4535)可能可以解决问题，我用的是`just with a make command instead of using the UnrealBuildTool`这个方法，然后再正常用`UE4Editor`的IDE打开。