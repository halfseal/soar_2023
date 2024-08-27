# given by command,
#   sudo docker pull stereolabs/zed:4.0-tools-devel-jetson-jp5.1.1
FROM stereolabs/zed:4.0-tools-devel-jetson-jp5.1.1

ENV LD_LIBRARY_PATH ${LD_LIBRARY_PATH}:/usr/lib/aarch64-linux-gnu/tegra
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_DISTRO=foxy
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3


# install ROS
RUN locale  # check for UTF-8
RUN sudo apt update && sudo apt install locales
RUN sudo locale-gen en_US en_US.UTF-8
RUN sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
RUN locale  # verify settings
RUN sudo apt install software-properties-common -y
RUN sudo add-apt-repository universe
RUN sudo apt update && sudo apt install curl -y
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN sudo apt update
RUN sudo apt upgrade -y
# install ROS ==> install foxy essentials
RUN sudo apt install ros-foxy-desktop python3-argcomplete -y
RUN sudo apt install ros-dev-tools -y
RUN sudo apt install ros-foxy-rmw-fastrtps* -y  
RUN sudo apt install ros-foxy-xacro -y
RUN sudo apt install ros-foxy-diagnostic-updater -y
RUN sudo apt install ros-foxy-*-msgs -y
RUN sudo apt install ros-foxy-robot-localization -y

RUN echo "Hello, World"

# install starter packs
RUN apt update
RUN apt install -y \
                    build-essential \
                    cmake \
                    git \
                    libbullet-dev \
                    python3-colcon-common-extensions \
                    python3-flake8 \
                    python3-pip \
                    python3-pytest-cov \
                    python3-rosdep \
                    python3-setuptools \
                    python3-vcstool \
                    wget

# install python packages
RUN python3 -m pip install -U \
                    argcomplete \
                    flake8-blind-except \
                    flake8-builtins \
                    flake8-class-newline \
                    flake8-comprehensions \
                    flake8-deprecated \
                    flake8-docstrings \
                    flake8-import-order \
                    flake8-quotes \
                    pytest-repeat \
                    pytest-rerunfailures \
                    pytest

# install dependencies
RUN sudo apt install --no-install-recommends \
                    libasio-dev \
                    libtinyxml2-dev \
                    libcunit1-dev

# set ROS bash settings
RUN echo "source /opt/ros/foxy/setup.bash" \
        "\nsource ~/soar_ws/install/local_setup.bash" \
        "\nsource /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" \
        "\nsource /usr/share/vcstool-completion/vcs.bash" \
        "\nsource /usr/share/colcon_cd/function/colcon_cd.sh" \
        "\nexport _colcon_cd_root=~/soar_ws" \
        "\nexport RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
        "\nexport RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}]: {message}'" \
        "\nexport RCUTILS_COLORIZED_OUTPUT=1" \
        "\nexport RCUTILS_LOGGING_USE_STDOUT=0" \
        "\nexport RCUTILS_LOGGING_BUFFERED_STREAM=1" \
        "\nalias cw='cd ~/soar_ws'" \
        "\nalias cs='cd ~/soar_ws/src'" \
        "\nalias ccd='colcon_cd'" \
        "\nalias cb='cd ~/robot_ws && colcon build --symlink-install'" \
        "\nalias cbs='colcon build --symlink-install'" \
        "\nalias cbp='colcon build --symlink-install --packages-select'" \
        "\nalias cbu='colcon build --symlink-install --packages-up-to'" \
        "\nalias ct='colcon test'" \
        "\nalias ctp='colcon test --packages-select'" \
        "\nalias ctr='colcon test-result'" \
        "\nalias rt='ros2 topic list'" \
        "\nalias re='ros2 topic echo'" \
        "\nalias rn='ros2 node list'" >> /root/.bashrc

# install nautilus, vim
RUN sudo apt install nautilus -y
RUN sudo apt install vim -y

# make workspace
RUN mkdir -p /root/soar_ws/src
WORKDIR /root/soar_ws
RUN . /opt/ros/foxy/setup.sh && \
    colcon build --symlink-install 
WORKDIR /root/soar_ws/install
RUN chmod +x ./local_setup.sh


# download px4_ros_com_ros2
RUN sudo apt install openjdk-11-jdk -y
WORKDIR /root
RUN git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 ~/Fast-RTPS-Gen
WORKDIR /root/Fast-RTPS-Gen/gradle/wrapper
RUN sed -i '3s|distributionUrl=.*|distributionUrl=https\://services.gradle.org/distributions/gradle-6.8.3-bin.zip|' /root/Fast-RTPS-Gen/gradle/wrapper/gradle-wrapper.properties
WORKDIR /root/Fast-RTPS-Gen
RUN ./gradlew assemble && sudo env "PATH=$PATH" ./gradlew install
RUN sudo apt install ros-foxy-eigen3-cmake-module
RUN sudo apt install python3-testresources -y
RUN sudo pip3 install -U empy pyros-genmsg setuptools


# build px4_ros_com_ros2
WORKDIR /root/soar_ws/src
RUN git clone -b release/1.13 https://github.com/PX4/px4_ros_com.git
RUN git clone -b release/1.13 https://github.com/PX4/px4_msgs.git
WORKDIR /root/soar_ws/src/px4_ros_com/scripts
RUN bash -c './build_ros2_workspace.bash'



# install zed wrapper
RUN sudo apt remove ros-foxy-image-transport-plugins ros-foxy-compressed-depth-image-transport ros-foxy-compressed-image-transport
WORKDIR /root/soar_ws/src
RUN git clone https://github.com/ros-perception/image_common.git --branch 3.0.0 --single-branch # clone the "v3.0.0" branch of the "image_common" repository
WORKDIR /root/soar_ws
RUN . /opt/ros/foxy/setup.sh && \
    colcon build --symlink-install
RUN apt remove ros-foxy-image-transport-plugins ros-foxy-compressed-depth-image-transport ros-foxy-compressed-image-transport
RUN . /opt/ros/foxy/setup.sh

WORKDIR /root/soar_ws/src
RUN git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
WORKDIR /root/soar_ws
RUN rosdep init
RUN rosdep update
RUN rosdep install --from-paths /root/soar_ws/src --ignore-src -r -y

RUN . /opt/ros/foxy/setup.sh && \
    colcon build --symlink-install --packages-up-to zed_wrapper --allow-overriding image_transport --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined" --parallel-workers $(nproc)

# build zed-rgb convert
WORKDIR /root/soar_ws/src
RUN git clone https://github.com/joony414/custom_zed_rgb_convert 
WORKDIR /root/soar_ws
RUN . /opt/ros/foxy/setup.sh && \
    colcon build --symlink-install --packages-up-to zed_rgb_convert --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined" --parallel-workers $(nproc)


# download custom yolo
WORKDIR /root/soar_ws/src
RUN git clone https://github.com/joony414/custom_yolov8_ros.git
RUN pip3 install -r /root/soar_ws/src/custom_yolov8_ros/requirements.txt
RUN pip3 uninstall torch -y
RUN pip3 uninstall torchvision -y
RUN pip3 install python-dateutil --upgrade
RUN apt-get install -y \
    autoconf \
    bc \
    build-essential \
    g++-8 \
    gcc-8 \
    clang-8 \
    lld-8 \
    gettext-base \
    gfortran-8 \
    iputils-ping \
    libbz2-dev \
    libc++-dev \
    libcgal-dev \
    libffi-dev \
    libfreetype6-dev \
    libhdf5-dev \
    libjpeg-dev \
    liblzma-dev \
    libncurses5-dev \
    libncursesw5-dev \
    libpng-dev \
    libreadline-dev \
    libssl-dev \
    libsqlite3-dev \
    libxml2-dev \
    libxslt-dev \
    locales \
    moreutils \
    openssl \
    python-openssl \
    rsync \
    scons \
    python3-pip \
    libopenblas-dev
ENV TORCH_INSTALL=https://developer.download.nvidia.cn/compute/redist/jp/v511/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install aiohttp numpy=='1.19.4' scipy=='1.5.3' && \
    python3 -m pip install --upgrade protobuf && \
    python3 -m pip install --no-cache $TORCH_INSTALL
WORKDIR /root
RUN git clone --branch v0.15.1 https://github.com/pytorch/vision torchvision
WORKDIR /root/torchvision
ENV BUILD_VERSION=0.15.1
RUN python3 setup.py install --user

# custom yolo8 build
WORKDIR /root/soar_ws
RUN rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/foxy/setup.sh && \
    colcon build --symlink-install --packages-up-to yolov8_bringup
RUN 


# custom usb_cam (gopro)
RUN sudo apt install ffmpeg v4l2loopback-dkms curl vlc -y
RUN sudo su -c "bash <(wget -qO- https://cutt.ly/PjNkrzq)" root
WORKDIR /root/soar_ws/src
RUN git clone https://github.com/joony414/custom_usb_cam
WORKDIR /root/soar_ws
RUN . /opt/ros/foxy/setup.sh && \
    colcon build --symlink-install --packages-up-to usb_cam