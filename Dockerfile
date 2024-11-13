# Docker file for aarch64 based Jetson device
ARG BASE_IMAGE="nvcr.io/nvidia/l4t-cuda:12.2.12-devel"
FROM ${BASE_IMAGE} AS base

# Store list of packages (must be first)
RUN mkdir -p /opt/nvidia/isaac_ros_dev_base && dpkg-query -W | sort > /opt/nvidia/isaac_ros_dev_base/aarch64-start-packages.csv

# Disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"]

# Ensure we have universe
RUN --mount=type=cache,target=/var/cache/apt \
apt-get update && apt-get install -y \
        software-properties-common \
&& add-apt-repository universe \
&& apt-get update

# Fundamentals
RUN --mount=type=cache,target=/var/cache/apt \
apt-get update && apt-get install -y \
    apt-utils \
    bash-completion \
    build-essential \
    ca-certificates \
    curl \
    git \
    git-lfs \
    gnupg2 \
    iputils-ping \
    libgoogle-glog-dev \
    locales \
    lsb-release \
    software-properties-common \
    sudo \
    tar \
    unzip \
    vim \
    wget \
    zlib1g-dev

# Add Isaac apt repository
RUN --mount=type=cache,target=/var/cache/apt \
    wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | apt-key add - && \
    grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" /etc/apt/sources.list || \
    echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" | tee -a /etc/apt/sources.list \
    && apt-get update

# Setup Jetson debian repositories
RUN --mount=type=cache,target=/var/cache/apt \
    apt-key adv --fetch-keys https://repo.download.nvidia.com/jetson/jetson-ota-public.asc \
    && apt-key adv --fetch-keys http://l4t-repo.nvidia.com/jetson-ota-internal.key \
    && echo 'deb https://repo.download.nvidia.com/jetson/common r36.3 main' > /etc/apt/sources.list.d/nvidia-l4t-apt-source.list \
    && echo 'deb https://repo.download.nvidia.com/jetson/t234 r36.3 main' >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list \
    && apt-get update

# Python basics
RUN --mount=type=cache,target=/var/cache/apt \
apt-get update && apt-get install -y \
    python3-dev \
    python3-distutils \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-venv \
    python3-zmq \
    python3.10 \
    python3.10-venv

# Set Python3 as default
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

# Core dev libraries
RUN --mount=type=cache,target=/var/cache/apt \
apt-get update && apt-get install -y \
    ffmpeg \
    gfortran \
    graphicsmagick-libmagick-dev-compat \
    jq \
    kmod \
    lcov \
    libasio-dev \
    libassimp-dev \
    libatlas-base-dev \
    libblas3 \
    libatlas3-base \
    libboost-all-dev \
    libboost-dev \
    libceres-dev \
    libbullet-dev \
    libcunit1-dev \
    libffi7 \
    libfreetype6 \
    libgraphicsmagick++1-dev \
    libhidapi-libusb0 \
    libinput10 \
    libjpeg8 \
    liblapack3 \
    libmnl0 \
    libmnl-dev \
    libncurses5-dev \
    libode-dev \
    libopenblas0 \
    libopencv-dev=4.5.4+dfsg-9ubuntu4 \
    libopenmpi3 \
    libpcap-dev \
    libpcl-dev \
    libsuitesparse-dev \
    libtinyxml2-dev \
    libturbojpeg \
    linuxptp \
    libunwind8 \
    libv4l-0 \
    libx264-dev \
    libxaw7-dev \
    libyaml-cpp-dev \
    llvm-14 \
    nlohmann-json3-dev \
    python3-opencv=4.5.4+dfsg-9ubuntu4 \
    python3-scipy

# Additional Python dependencies
RUN python3 -m pip install -U \
    Cython \
    pymongo \
    wheel \
    scikit-learn \
    ninja \
    networkx \
    "numpy>=1.24.4,<2" \
    numpy-quaternion \
    pyyaml \
    "setuptools_scm>=6.2" \
    trimesh \
    "yourdfpy>=0.0.53" \
    "warp-lang>=0.9.0" \
    "scipy>=1.7.0" \
    tqdm \
    importlib_resources


# Update environment
RUN update-alternatives --install /usr/bin/llvm-config llvm-config /usr/bin/llvm-config-14 14
ENV LD_LIBRARY_PATH="/opt/nvidia/vpi3/lib64:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/tegra:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda-12.2/targets/aarch64-linux/lib:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/tegra-egl:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/tegra/weston:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/usr/lib/aarch64-linux-gnu-host"
ENV PATH="/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/src/tensorrt/bin:${PATH}"

# Install CUDA packages
RUN --mount=type=cache,target=/var/cache/apt \
apt-get update && apt-get install -y --no-install-recommends \
    cuda-cudart-12-2 \
    cuda-libraries-12-2 \
    cuda-nvml-dev-12-2 \
    cuda-sanitizer-12-2 \
    cuda-toolkit-12-2 \
    libcublas-12-2 \
    libcudnn8 \
    libcusparse-12-2 \
    libnpp-12-2

# Install TensorRT and VPI
RUN --mount=type=cache,target=/var/cache/apt \
mkdir -p /lib/firmware && \
apt-get update && apt-get install -y \
    libnvvpi3 \
    tensorrt \
    vpi3-dev

# Install Tao converter
RUN mkdir -p /opt/nvidia/tao && cd /opt/nvidia/tao && \
    wget --content-disposition 'https://api.ngc.nvidia.com/v2/resources/org/nvidia/team/tao/tao-converter/v5.1.0_jp6.0_aarch64/files?redirect=true&path=tao-converter' -O tao-converter && \
    chmod 755 tao-converter

ENV PATH="${PATH}:/opt/nvidia/tao"
ENV TRT_LIB_PATH="/usr/lib/aarch64-linux-gnu"
ENV TRT_INCLUDE_PATH="/usr/include/aarch64-linux-gnu"

# PyTorch (NV CUDA edition)
# https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html
RUN python3 -m pip install --no-cache \
        https://developer.download.nvidia.com/compute/redist/jp/v60dp/pytorch/torch-2.2.0a0+6a974be.nv23.11-cp310-cp310-linux_aarch64.whl

# Install Triton server from https://github.com/triton-inference-server/server/releases/tag/v2.40.0
RUN --mount=type=cache,target=/var/cache/apt \
apt-get update && apt-get install -y --no-install-recommends \
    libb64-0d \
    libre2-9 \
    rapidjson-dev \
    libopenblas-dev \
    libarchive-dev

RUN --mount=type=cache,target=/var/cache/apt \
    cd /opt \
    && wget https://github.com/triton-inference-server/server/releases/download/v2.40.0/tritonserver2.40.0-igpu.tar.gz \
    && tar -xzvf tritonserver2.40.0-igpu.tar.gz \
    && chmod 644 /opt/tritonserver/backends/tensorflow/libtensorflow_cc.so.2 \
    && chmod 644 /opt/tritonserver/backends/tensorflow/libtensorflow_framework.so.2 \
    && rm tritonserver2.40.0-igpu.tar.gz

ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/tritonserver/lib"

# Install boost version >= 1.78 for boost::span
# Current libboost-dev apt packages are < 1.78, so install from tar.gz
RUN --mount=type=cache,target=/var/cache/apt \
    wget -O /tmp/boost.tar.gz \
    https://boostorg.jfrog.io/artifactory/main/release/1.80.0/source/boost_1_80_0.tar.gz \
    && (cd /tmp && tar xzf boost.tar.gz) \
    && cd /tmp/boost_1_80_0 \
    && ./bootstrap.sh --prefix=/usr \
    && ./b2 install \
    && rm -rf /tmp/boost*

# Install CV-CUDA
RUN --mount=type=cache,target=/var/cache/apt \
    cd /tmp && \
    wget https://github.com/CVCUDA/CV-CUDA/releases/download/v0.5.0-beta/nvcv-lib-0.5.0_beta_DP-cuda12-aarch64-linux.deb && \
    dpkg -i nvcv-lib-0.5.0_beta_DP-cuda12-aarch64-linux.deb && \
    wget https://github.com/CVCUDA/CV-CUDA/releases/download/v0.5.0-beta/nvcv-dev-0.5.0_beta_DP-cuda12-aarch64-linux.deb && \
    dpkg -i nvcv-dev-0.5.0_beta_DP-cuda12-aarch64-linux.deb

# Add MQTT binaries and libraries
RUN --mount=type=cache,target=/var/cache/apt \
apt-add-repository ppa:mosquitto-dev/mosquitto-ppa \
&& apt-get update && apt-get install -y \
        mosquitto \
        mosquitto-clients

# Install jtop
RUN python3 -m pip install -U \
    jetson-stats

# Store list of packages (must be last)
RUN mkdir -p /opt/nvidia/isaac_ros_dev_base && dpkg-query -W | sort > /opt/nvidia/isaac_ros_dev_base/aarch64-end-packages.csv






#########################################
############# BUILD FOR ROS2 ############
#########################################
FROM base AS ros2

# Store list of packages (must be first)
RUN mkdir -p /opt/nvidia/isaac_ros_dev_base && dpkg-query -W | sort > /opt/nvidia/isaac_ros_dev_base/ros2_humble-start-packages.csv

# disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"]

# Env setup
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV ROS_PYTHON_VERSION=3
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Avoid setup.py and easy_install deprecation warnings caused by colcon and setuptools
# https://github.com/colcon/colcon-core/issues/454
ENV PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources,ignore:::setuptools.command.develop
RUN echo "Warning: Using the PYTHONWARNINGS environment variable to silence setup.py and easy_install deprecation warnings caused by colcon"

# Add ROS 2 apt repository
RUN --mount=type=cache,target=/var/cache/apt \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update

# ROS fundamentals
RUN --mount=type=cache,target=/var/cache/apt \
apt-get update && apt-get install -y \
        devscripts \
        dh-make \
        fakeroot \
        libxtensor-dev \
        python3-bloom \
        python3-colcon-common-extensions \
        python3-pip \
        python3-pybind11 \
        python3-pytest-cov \
        python3-rosdep \
        python3-rosinstall-generator \
        python3-vcstool \
        quilt

# ROS Python fundamentals
RUN python3 -m pip install -U \
        flake8-blind-except \
        flake8-builtins \
        flake8-class-newline \
        flake8-comprehensions \
        flake8-deprecated \
        flake8-docstrings \
        flake8-import-order \
        flake8-quotes \
        matplotlib \
        pandas \
        rosbags \
        setuptools==65.7.0

# Install ROS 2 Humble
RUN --mount=type=cache,target=/var/cache/apt \
apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-angles \
    ros-humble-apriltag \
    ros-humble-behaviortree-cpp-v3 \
    ros-humble-bondcpp \
    ros-humble-camera-calibration-parsers \
    ros-humble-camera-info-manager \
    ros-humble-compressed-image-transport \
    ros-humble-compressed-depth-image-transport \
    ros-humble-cv-bridge \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py \
    ros-humble-diagnostics \
    ros-humble-diagnostic-aggregator \
    ros-humble-diagnostic-updater \
    ros-humble-example-interfaces \
    ros-humble-foxglove-bridge \
    ros-humble-image-geometry \
    ros-humble-image-pipeline \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-launch-xml \
    ros-humble-launch-yaml \
    ros-humble-launch-testing \
    ros-humble-launch-testing-ament-cmake \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-nav2-mppi-controller \
    ros-humble-nav2-graceful-controller \
    ros-humble-navigation2 \
    ros-humble-ompl \
    ros-humble-resource-retriever \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-compression-zstd \
    ros-humble-rosbag2-cpp \
    ros-humble-rosbag2-py \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-rosbridge-suite \
    ros-humble-rosx-introspection \
    ros-humble-rqt-graph \
    ros-humble-rqt-image-view \
    ros-humble-rqt-reconfigure \
    ros-humble-rqt-robot-monitor \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-sensor-msgs \
    ros-humble-slam-toolbox \
    ros-humble-v4l2-camera \
    ros-humble-vision-opencv \
    ros-humble-vision-msgs \
    ros-humble-vision-msgs-rviz-plugins

# Setup rosdep
COPY docker/rosdep/extra_rosdeps.yaml /etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml
RUN --mount=type=cache,target=/var/cache/apt \
    rosdep init \
    && echo "yaml file:///etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml" | tee /etc/ros/rosdep/sources.list.d/00-nvidia-isaac.list \
    && sed -i 's|gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml fuerte||g' /etc/ros/rosdep/sources.list.d/20-default.list \
    && rosdep update

####### -- Install updated packages over installed debians

# Install negotiated from source
RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
    && git clone https://github.com/osrf/negotiated && cd negotiated && git checkout master \
    && source ${ROS_ROOT}/setup.bash \
    && cd negotiated_interfaces && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
    && cd negotiated && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y ./*.deb && rm ./*.deb

# Install image_proc from 55bf2a38 with backported resize node fix
# https://github.com/ros-perception/image_pipeline/pull/786/commits/969d6c763df99b42844742946f7a70c605a72a15
# Revert breaking QoS changes in https://github.com/ros-perception/image_pipeline/pull/814
RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
    && git clone https://github.com/ros-perception/image_pipeline.git && cd image_pipeline && git checkout 55bf2a38c327b829c3da444f963a6c66bfe0598f \
    && git config user.email "builder@nvidia.com" && git config user.name "NVIDIA Builder" \
    && git remote add fork https://github.com/schornakj/image_pipeline.git && git fetch fork && git cherry-pick 969d6c763df99b42844742946f7a70c605a72a15 \
    && source ${ROS_ROOT}/setup.bash \
    && cd image_proc && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y --allow-downgrades ./*.deb \
    && echo "image_pipeline (image_proc) https://github.com/ros-perception/image_pipeline/pull/786/commits/969d6c763df99b42844742946f7a70c605a72a15 on 55bf2a38" >> ${ROS_ROOT}/VERSION \
    && cd ../ && rm -Rf src build log

# Install patched rclcpp package with backported multithreadedexecutor fix
# https://github.com/ros2/rclcpp/commit/232262c02a1265830c7785b7547bd51e1124fcd8
COPY docker/patches/rclcpp-disable-tests.patch /tmp/
RUN --mount=type=cache,target=/var/cache/apt \
    mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
    && export RCLCPP_VERSION="release/humble/rclcpp/$(apt-cache madison ros-humble-rclcpp | grep -m1 -oP 'ros-humble-rclcpp \| \K[^j]+(?=jammy)')" \
    && echo ${RCLCPP_VERSION} \
    && git clone https://github.com/ros2-gbp/rclcpp-release.git && cd rclcpp-release && git checkout ${RCLCPP_VERSION} \
    && patch -i /tmp/rclcpp-disable-tests.patch \
    && unset RCLCPP_VERSION \
    && git config user.email "builder@nvidia.com" && git config user.name "NVIDIA Builder" \
    && git remote add rclcpp https://github.com/ros2/rclcpp.git && git fetch rclcpp \
    && git cherry-pick 232262c02a1265830c7785b7547bd51e1124fcd8 \
    && source ${ROS_ROOT}/setup.bash \
    && cd ../ && rosdep install -i -r --from-paths rclcpp-release/ --rosdistro humble -y \
    && cd rclcpp-release && bloom-generate rosdebian && fakeroot debian/rules binary \
    && cd ../ && apt-get install -y --allow-downgrades ./*.deb \
    && echo "rclcpp https://github.com/ros2/rclcpp/commit/232262c02a1265830c7785b7547bd51e1124fcd8" >> ${ROS_ROOT}/VERSION \
    && cd ../ && rm -Rf src build log

# # Install Moveit 2 ROS packages
# RUN --mount=type=cache,target=/var/cache/apt \
# apt-get update && apt-get install -y \
#     ros-humble-ament-cmake \
#     ros-humble-ament-cmake-gtest \
#     ros-humble-control-msgs \
#     ros-humble-controller-manager \
#     ros-humble-geometric-shapes \
#     ros-humble-gripper-controllers \
#     ros-humble-interactive-markers \
#     ros-humble-joint-state-broadcaster \
#     ros-humble-joint-state-publisher \
#     ros-humble-joint-trajectory-controller \
#     ros-humble-joy \
#     ros-humble-launch-param-builder \
#     ros-humble-moveit \
#     ros-humble-moveit-common \
#     ros-humble-moveit-configs-utils \
#     ros-humble-moveit-core \
#     ros-humble-moveit-msgs \
#     ros-humble-moveit-ros-perception \
#     ros-humble-moveit-ros-planning \
#     ros-humble-moveit-ros-planning-interface \
#     ros-humble-moveit-servo \
#     ros-humble-moveit-visual-tools \
#     ros-humble-pluginlib \
#     ros-humble-py-binding-tools \
#     ros-humble-robot-state-publisher \
#     ros-humble-ros2-control \
#     ros-humble-rviz-visual-tools \
#     ros-humble-rviz2 \
#     ros-humble-srdfdom \
#     ros-humble-tf2-eigen \
#     ros-humble-tf2-geometry-msgs \
#     ros-humble-tf2-ros \
#     ros-humble-topic-based-ros2-control \
#     ros-humble-ur-description \
#     ros-humble-ur-moveit-config \
#     ros-humble-ur-msgs \
#     ros-humble-xacro

# # Install various moveit_resources packages from source.
# RUN --mount=type=cache,target=/var/cache/apt \
#     mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
#     && git clone https://github.com/ros-planning/moveit_resources.git -b humble \
#     && cd moveit_resources && source ${ROS_ROOT}/setup.bash \
#     && cd fanuc_description && bloom-generate rosdebian && fakeroot debian/rules binary \
#     && cd .. && apt-get install -y ./*.deb && rm *.deb \
#     && cd fanuc_moveit_config && bloom-generate rosdebian && fakeroot debian/rules binary \
#     && cd .. && apt-get install -y ./*.deb && rm *.deb \
#     && cd panda_description && bloom-generate rosdebian && fakeroot debian/rules binary \
#     && cd .. && apt-get install -y ./*.deb && rm *.deb \
#     && cd panda_moveit_config && bloom-generate rosdebian && fakeroot debian/rules binary \
#     && cd .. && apt-get install -y ./*.deb && rm *.deb \
#     && cd pr2_description && bloom-generate rosdebian && fakeroot debian/rules binary \
#     && cd .. && apt-get install -y ./*.deb && rm *.deb \
#     && cd moveit_resources && bloom-generate rosdebian && fakeroot debian/rules binary \
#     && cd .. && apt-get install -y ./*.deb && rm *.deb
#
# # Install MoveIt task constructor from source.  The "demo" package depends on moveit_resources_panda_moveit_config,
# # installed from source above.
# RUN --mount=type=cache,target=/var/cache/apt \
#     mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
#     && git clone https://github.com/ros-planning/moveit_task_constructor.git -b humble \
#     && cd moveit_task_constructor && source ${ROS_ROOT}/setup.bash \
#     && cd msgs && bloom-generate rosdebian && fakeroot debian/rules binary \
#     && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
#     && cd rviz_marker_tools && bloom-generate rosdebian && fakeroot debian/rules binary \
#     && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
#     && cd core && bloom-generate rosdebian && fakeroot debian/rules binary DEB_BUILD_OPTIONS=nocheck \
#     && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
#     && cd capabilities && bloom-generate rosdebian && fakeroot debian/rules binary DEB_BUILD_OPTIONS=nocheck \
#     && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
#     && cd visualization && bloom-generate rosdebian && fakeroot debian/rules binary DEB_BUILD_OPTIONS=nocheck \
#     && cd ../ && apt-get install -y ./*.deb && rm ./*.deb \
#     && cd demo && bloom-generate rosdebian && fakeroot debian/rules binary DEB_BUILD_OPTIONS=nocheck \
#     && cd ../ && apt-get install -y ./*.deb && rm ./*.deb
#
# # MoveIt 2's hybrid planning package depends on moveit_resources_panda_moveit_config, installed from source above.
# RUN --mount=type=cache,target=/var/cache/apt \
# apt-get update && apt-get install -y \
#     ros-humble-moveit-hybrid-planning
#
# # Install moveit2_tutorials from source (depends on moveit_hybrid_planning).
# RUN --mount=type=cache,target=/var/cache/apt \
#     mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
#     && git clone https://github.com/ros-planning/moveit2_tutorials.git -b humble \
#     && cd moveit2_tutorials && source ${ROS_ROOT}/setup.bash \
#     && bloom-generate rosdebian && fakeroot debian/rules binary \
#     && cd ../ && apt-get install -y ./*.deb && rm ./*.deb

# Install paho-mqtt for isaac_ros_mission_client
RUN python3 -m pip install -U \
        paho-mqtt==1.6.1

# Patch gtest to make it work with CXX 17
RUN sudo sed -i '917i #ifdef GTEST_INTERNAL_NEED_REDUNDANT_CONSTEXPR_DECL' /usr/src/googletest/googletest/include/gtest/internal/gtest-internal.h \
    && sudo sed -i '920i #endif' /usr/src/googletest/googletest/include/gtest/internal/gtest-internal.h \
    && sudo sed -i '2392i #if defined(GTEST_INTERNAL_CPLUSPLUS_LANG) && \\\n    GTEST_INTERNAL_CPLUSPLUS_LANG < 201703L\n#define GTEST_INTERNAL_NEED_REDUNDANT_CONSTEXPR_DECL 1\n#endif' \
    /usr/src/googletest/googletest/include/gtest/internal/gtest-port.h



# Store list of packages (must be last)
RUN mkdir -p /opt/nvidia/isaac_ros_dev_base && dpkg-query -W | sort > /opt/nvidia/isaac_ros_dev_base/ros2_humble-end-packages.csv




########################################
######### BUILD REALSENSE ##############
########################################

FROM ros2 AS ros2_rs

ARG LIBREALSENSE_SOURCE_VERSION=v2.55.1

COPY docker/scripts/build-librealsense.sh /opt/realsense/build-librealsense.sh
COPY docker/scripts/install-realsense-dependencies.sh /opt/realsense/install-realsense-dependencies.sh

RUN chmod +x /opt/realsense/install-realsense-dependencies.sh && \
    /opt/realsense/install-realsense-dependencies.sh; \
    chmod +x /opt/realsense/build-librealsense.sh && /opt/realsense/build-librealsense.sh -v ${LIBREALSENSE_SOURCE_VERSION};

# Copy hotplug script for udev rules/hotplug for RealSense
RUN mkdir -p /opt/realsense/
COPY docker/scripts/hotplug-realsense.sh /opt/realsense/hotplug-realsense.sh
COPY docker/udev_rules/99-realsense-libusb-custom.rules /etc/udev/rules.d/99-realsense-libusb-custom.rules


# ########################################
# ############ BUILD PX4 #################
# ########################################
#
FROM ros2_rs AS ros2_rs_px4

RUN apt-get update && apt-get install -y --no-install-recommends \
  git \
  ninja-build \
  pkg-config \
  gcc \
  g++ \
  systemd \
  python3-pip \
  && rm -rf /var/lib/apt/lists/*

# install python dependencies
RUN python3 -m pip install empy==3.3.4 pyros-genmsg setuptools meson

# install mavlink router
WORKDIR /root
RUN git clone --depth 1 --branch v4 --recursive https://github.com/intel/mavlink-router.git
WORKDIR /root/mavlink-router
# RUN git submodule update --init --recursive
RUN meson setup build . --buildtype=release \
  && ninja -C build \
  && ninja -C build install

# install microXRCE agent
WORKDIR /root
RUN git clone --depth 1 --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
WORKDIR /root/Micro-XRCE-DDS-Agent/build
RUN cmake -DCMAKE_BUILD_TYPE=Release ..  && make -j$(($(nproc) - 1)) && make install -j$(($(nproc) - 1))
RUN ldconfig /usr/local/lib/

########################################
######### BUILD WORKSPACE ##############
########################################

FROM ros2_rs_px4 AS ros2_rs_px4_vslam

# install some additional dependencies and vslam
RUN --mount=type=cache,target=/var/cache/apt \
  apt-get update && apt-get install -y \
  iputils-ping \
  libboost-all-dev \
  ros-${ROS_DISTRO}-diagnostics \
  ros-${ROS_DISTRO}-rqt-robot-monitor \
  ros-${ROS_DISTRO}-isaac-ros-visual-slam \
  ros-${ROS_DISTRO}-isaac-ros-visual-slam-interfaces

# fix setuptools
RUN pip3 uninstall setuptools_scm -y && pip3 install setuptools

#########################################
######### INSTALL EIGEN3.3.7 ############
#########################################
WORKDIR /root
RUN git clone https://gitlab.com/libeigen/eigen.git --branch 3.3.7 --single-branch
WORKDIR /root/eigen/build
RUN cmake .. && make && make install


RUN --mount=type=cache,target=/var/cache/apt \
  apt-get update && apt-get install -y \
  ros-${ROS_DISTRO}-pcl-ros

##########################################
########### FINALIZE #####################
##########################################

FROM ros2_rs_px4_vslam AS ros2_ws

# install the RTPS profile
COPY ./docker/middleware_profiles/rtps_udp_profile.xml /usr/local/share/middleware_profiles/rtps_udp_profile.xml

WORKDIR /root/ros_ws
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros_ws/install/setup.bash" >> ~/.bashrc
