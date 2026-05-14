FROM nvidia/cuda:12.6.3-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i 's|http://archive.ubuntu.com/ubuntu/|http://br.archive.ubuntu.com/ubuntu/|g' /etc/apt/sources.list && \
    sed -i 's|http://security.ubuntu.com/ubuntu/|http://br.archive.ubuntu.com/ubuntu/|g' /etc/apt/sources.list

ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USER_NAME=booster

RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && \
    apt-get install -y \
    ros-humble-desktop \
    ros-humble-swri-console \
    git \
    gedit \
    python3-pip \
    nano \
    ros-humble-rviz2 \
    libzmq3-dev \
    wget \
    g++ \
    zlib1g-dev libcurl4-openssl-dev \
    libnvinfer-dev=8.6.1.6-1+cuda12.0 \
    libnvinfer-plugin-dev=8.6.1.6-1+cuda12.0 \
    libnvparsers-dev=8.6.1.6-1+cuda12.0 \
    libnvinfer-headers-dev=8.6.1.6-1+cuda12.0 \
    libnvinfer-headers-plugin-dev=8.6.1.6-1+cuda12.0 \
    python3-dev \
    libgoogle-glog-dev \
    ros-humble-backward-ros \
    libopenblas-dev \
    psmisc \
    nlohmann-json3-dev \
    unzip \
    sudo && \
    apt-mark hold libnvinfer-dev libnvinfer-plugin-dev libnvparsers-dev libnvinfer-headers-dev libnvinfer-headers-plugin-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN groupadd --gid $GROUP_ID $USER_NAME && \
    useradd --uid $USER_ID --gid $GROUP_ID --create-home --shell /bin/bash $USER_NAME

RUN usermod -aG sudo $USER_NAME && \
    echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN pip3 install -U colcon-common-extensions

RUN cd /tmp && \
    git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git && \
    cd BehaviorTree.CPP && \
    git checkout 4.6.2 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig

RUN git clone https://github.com/BoosterRobotics/booster_robotics_sdk.git /tmp/booster_robotics_sdk && \
    cd /tmp/booster_robotics_sdk && \
    chmod +x install.sh && \
    ./install.sh && \
    cd / && \
    rm -rf /tmp/booster_robotics_sdk

RUN cd /tmp && \
    git clone https://github.com/NVIDIA/TensorRT.git && \
    cd TensorRT && \
    git checkout v10.3.0 && \
    git submodule update --init --recursive && \
    mkdir -p build && cd build && \
    cmake .. -DTRT_OUT_DIR=$PWD/out && \
    make -j$(nproc) && \
    mkdir -p /usr/local/tensorrt-10.3 && \
    cp -r out/* /usr/local/tensorrt-10.3/ && \
    cp -r ../include /usr/local/tensorrt-10.3/ && \
    cp -r ../parsers /usr/local/tensorrt-10.3/include/

RUN echo 'export LD_LIBRARY_PATH=/usr/local/tensorrt-10.3/lib:$LD_LIBRARY_PATH' >> ~/.bashrc && \
    echo 'export PATH=/usr/local/tensorrt-10.3/bin:$PATH' >> ~/.bashrc

RUN cd /tmp && \
    wget https://github.com/rerun-io/rerun/releases/download/0.21.0/rerun_cpp_sdk.zip && \
    unzip rerun_cpp_sdk.zip && \
    cd rerun_cpp_sdk && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd / && \
    rm -rf /tmp/rerun_cpp_sdk.zip /tmp/rerun_cpp_sdk && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/${USER_NAME}/booster_ws

COPY --chown=${USER_NAME}:${USER_NAME} scripts/ ./scripts/
COPY --chown=${USER_NAME}:${USER_NAME} src/ ./src/
COPY --chown=${USER_NAME}:${USER_NAME} configs/ ./configs/
COPY internal_sdk /usr/local/include/booster_internal

RUN chown -R booster:booster /home/booster

USER $USER_NAME

ENV LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}

RUN bash -c "source /opt/ros/humble/setup.bash && ./scripts/build.sh"
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USER_NAME}/.bashrc && \
    echo "source /home/${USER_NAME}/booster_ws/install/setup.bash" >> /home/${USER_NAME}/.bashrc


CMD ["bash"]