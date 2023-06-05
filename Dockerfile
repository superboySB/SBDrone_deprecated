FROM px4io/px4-dev-ros-melodic:latest

ENV java_version=11
ENV gazebo_classic_version=9

RUN echo "Running within docker, installing initial dependencies"
RUN sed -i s@/archive.ubuntu.com/@/mirrors.aliyun.com/@g /etc/apt/sources.list
RUN apt-get update && apt-get -y install ca-certificates gnupg lsb-core sudo wget \
    astyle build-essential cmake cppcheck file g++ gcc gdb git lcov libfuse2 \ 
    libxml2-dev libxml2-utils make ninja-build python3 python3-dev python3-pip \
	python3-setuptools python3-wheel rsync shellcheck unzip zip bc ant \
    openjdk-$java_version-jre openjdk-$java_version-jdk libvecmath-java

RUN echo "Installing PX4 general dependencies"
RUN python3 -m pip install --user argcomplete argparse>=1.2 cerberus coverage \
    empy>=3.3 future jinja2>=2.8 jsonschema kconfiglib lxml matplotlib>=3.0.* \
    numpy>=1.13 nunavut>=1.1.0 packaging pandas>=0.21 pkgconfig psutil pygments \
    wheel>=0.31.1 pymavlink pyros-genmsg pyserial pyulog>=0.5.0 pyyaml requests \
    setuptools>=39.2.0 six>=1.12.0 toml>=0.9 sympy

RUN echo "Installing PX4 simulation dependencies"
RUN update-alternatives --set java $(update-alternatives --list java | grep "java-$java_version")
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \ 
    apt-get update -y
ENV gazebo_packages="gazebo$gazebo_classic_version libgazebo$gazebo_classic_version-dev"
RUN apt-get -y --quiet --no-install-recommends install dmidecode $gazebo_packages \
	gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
	gstreamer1.0-plugins-ugly gstreamer1.0-libav libeigen3-dev libgstreamer-plugins-base1.0-dev \
	libimage-exiftool-perl libopencv-dev libxml2-utils pkg-config protobuf-compiler
    