# SBDrone
use sim-to-real RL to achieve a perception-aware velocity controller. This is note for runing codes in x86_64 machines

# Configure the enironment
## Install dependencies
```sh
sudo apt-get update && sudo apt-get install -y --no-install-recommends build-essential cmake libzmqpp-dev libopencv-dev libgoogle-glog-dev protobuf-compiler ros-$ROS_DISTRO-octomap-msgs ros-$ROS_DISTRO-octomap-ros ros-$ROS_DISTRO-joy python3-vcstool python-catkin-tools git python3-pip lsb-core vim gedit locate wget desktop-file-utils python3-empy gcc g++ cmake git gnuplot doxygen graphviz software-properties-common apt-transport-https curl libqglviewer-dev-qt5 libzmqpp-dev libeigen3-dev libglfw3-dev libglm-dev libvulkan1 vulkan-utils gdb  libsdl-image1.2-dev libsdl-dev ros-melodic-octomap-mapping libomp-dev libompl-dev ompl-demos && curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add - && sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" && sudo apt update && sudo apt install code -y && sudo pip3 install catkin-tools numpy -i https://pypi.tuna.tsinghua.edu.cn/simple
```

## Install Open3D
```sh
tar -C ~/ -zxvf ~/dependencies/Open3D.tgz && cd ~/Open3D/ && util/scripts/install-deps-ubuntu.sh assume-yes && mkdir build && cd build && cmake -DBUILD_SHARED_LIBS=ON .. && make -j16 && sudo make install
```

## Install cv_bridge
```sh
mkdir -p ~/cv_bridge_ws/src && tar -C ~/cv_bridge_ws/src/ -zxvf ~/dependencies/vision_opencv.tgz && apt-cache show ros-melodic-cv-bridge | grep Version && cd ~/cv_bridge_ws/ && catkin config --install && catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so && catkin build && source install/setup.bash --extend
```

---------

## Install Python Package:
```sh
sudo pip3 install --upgrade pip && pip3 install tensorflow-gpu==1.14 markupsafe scikit-build -i https://pypi.tuna.tsinghua.edu.cn/simple && cd ~/flightmare_ws/src/flightmare/flightlib && pip3 install -e . -i https://pypi.tuna.tsinghua.edu.cn/simple
```




## Compile our project
**Every time when you change the code in other machines**, you can delete the project and then restart by:
```sh
cd ~ && git clone https://github.com/superboySB/flightmare_ws.git
```
```sh
echo "export FLIGHTMARE_PATH=~/flightmare_ws/src/flightmare" >> ~/.bashrc && source ~/.bashrc
```
Download the Flightmare Unity Binary **RPG_Flightmare.tar.xz** for rendering from the [Releases](https://github.com/uzh-rpg/flightmare/releases) and extract it into the /home/qiyuan/flightmare_ws/src/flightmare/flightrender/
```sh
cd ~/flightmare_ws/ && catkin init && catkin config --extend /opt/ros/melodic && catkin config --merge-devel && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-fdiagnostics-color && catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so && catkin build
```

## Install Python Package: flightlib + flightrl
flightlib
```sh
sudo pip3 install --upgrade pip && pip3 install tensorflow-gpu==1.14 markupsafe scikit-build -i https://pypi.tuna.tsinghua.edu.cn/simple && cd ~/flightmare_ws/src/flightmare/flightlib && pip3 install -e . -i https://pypi.tuna.tsinghua.edu.cn/simple
```
flightrl (main)
```sh
cd ~/flightmare_ws/src/flightmare/flightrl && pip3 install -e . -i https://pypi.tuna.tsinghua.edu.cn/simple
```

# Basic Usage with ROS
## Launch Flightmare (use gazebo-based dynamics)
In this example, we show how to use the [RotorS](https://github.com/ethz-asl/rotors_simulator) for the quadrotor dynamics modelling, [rpg_quadrotor_control](https://github.com/uzh-rpg/rpg_quadrotor_control) for model-based controller, and Flightmare for image rendering.
```sh
cd ~/flightmare_ws && source ./devel/setup.bash && roslaunch flightros rotors_gazebo.launch
```
We hope this example can serve as a starting point for many other applications. For example, Flightmare can be used with other multirotor models that comes with RotorS such as AscTec Hummingbird, the AscTec Pelican, or the AscTec Firefly. The default controller in [rpg_quadrotor_control](https://github.com/uzh-rpg/rpg_quadrotor_control) is a PID controller. Users have the option to use more advanced controller in this framework, such as [Perception-Aware Model Predictive Control](https://github.com/uzh-rpg/rpg_mpc).

# Basic Usage with Python
## Train neural network controller using PPO
```sh
cd ~/flightmare_ws/examples && python3 run_drone_control.py --train 1
```

## Test a pre-trained neural network controller
```sh
cd ~/flightmare_ws/examples && python3 run_drone_control.py --train 0
```

## With Unity Rendering
To enable unity for visualization, double click the extracted executable file RPG_Flightmare.x84-64
```sh
~/flightmare_ws/src/flightmare/flightrender/RPG_Flightmare.x86_64
```
and then test a pre-trained controller
```sh
cd ~/flightmare_ws/examples && python3 run_drone_control.py --train 0 --render 1
```



