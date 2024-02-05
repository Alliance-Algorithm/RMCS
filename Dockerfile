# Base container, provides a runtime environment
FROM ros:humble AS rmcs-base

WORKDIR /

# Change bash as default shell instead of sh
SHELL ["/bin/bash", "-c"]

# Install some tools, eigen and ceres installation dependencies.
RUN apt-get update && apt-get -y install \
    vim wget curl unzip \
    build-essential \
    cmake \
    make ninja-build \
    libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ceres 2.2.0
RUN mkdir ceres_tmp && cd ceres_tmp && \
    wget http://ceres-solver.org/ceres-solver-2.2.0.tar.gz && \
    tar zxf ceres-solver-2.2.0.tar.gz && \
    mkdir build && cd build && \
    cmake ../ceres-solver-2.2.0 && \
    make -j8 && \
    make install && \
    cd ../.. && rm -rf ./ceres_tmp

# Install openvino 2023.3 runtime (C++ API only)
RUN wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    echo "deb https://apt.repos.intel.com/openvino/2023 ubuntu22 main" | tee /etc/apt/sources.list.d/intel-openvino-2023.list && \
    apt-get update && \
    apt-get install -y openvino-2023.3.0 && \
    rm -rf /var/lib/apt/lists/* && \
    rm ./GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB

# Install HikCamera runtime SDK
RUN mkdir mvs_tmp && cd mvs_tmp && \
    wget https://www.hikrobotics.com/cn2/source/support/software/MvCamCtrlSDK_STD_V4.3.0_231225.zip && \
    unzip MvCamCtrlSDK_STD_V4.3.0_231225.zip && \
    echo -e '\n' | dpkg -i ./MvCamCtrlSDK_Runtime-4.3.0_x86_64_20231225.deb && \
    cd .. && rm -rf ./mvs_tmp

# Install opencv 4.9.0 with contrib
RUN mkdir opencv_tmp && cd opencv_tmp && \
    wget -O opencv.tar.gz https://github.com/opencv/opencv/archive/refs/tags/4.9.0.tar.gz && \
    wget -O opencv_contrib.tar.gz https://github.com/opencv/opencv_contrib/archive/refs/tags/4.9.0.tar.gz && \
    tar -zxf ./opencv.tar.gz && tar -zxf ./opencv_contrib.tar.gz && \
    mkdir build && cd build && \
    cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.9.0/modules ../opencv-4.9.0 && \
    make -j8 && make install && \
    cd ../.. && rm -rf ./opencv_tmp

# Download RMCS source, fix dependencies and compile
RUN git clone --recurse-submodules https://github.com/Alliance-Algorithm/RMCS && \
    cd RMCS/rmcs_ws && \
    source /opt/ros/humble/local_setup.bash && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/* && \
    colcon build && \
    mv ./install /rmcs-install && cd / && rm -rf RMCS


# Runtime container, will automatically launch the main program
FROM rmcs-base AS rmcs-runtime

RUN echo '#!/usr/bin/bash' > /entrypoint.bash && \
    echo 'source /opt/ros/humble/setup.bash' >> /entrypoint.bash && \
    echo 'source /rmcs-install/setup.bash' >> /entrypoint.bash && \
    echo 'ros2 run rmcs_controller rmcs_controller' >> /entrypoint.bash && \
    chmod +x /entrypoint.bash

ENTRYPOINT [ "/entrypoint.bash" ]


# Developing container, works with devcontainer
FROM rmcs-base AS rmcs-develop

# Install develop tools (clangd/zsh/etc...)
RUN apt-get update && apt-get -y install \
    lsb-release software-properties-common gnupg zsh sudo && \
    echo -e "\n" | bash -c "$(wget -O - https://apt.llvm.org/llvm.sh)" && \
    ln -s /usr/bin/clangd-* /usr/bin/clangd

RUN apt-get -y install libcanberra-gtk-module libcanberra-gtk3-module

# Add user
RUN useradd -m developer --shell /bin/zsh && echo "developer:developer" | chpasswd && adduser developer sudo && \
    echo "developer ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set user
USER developer
WORKDIR /home/developer
ENV USER=developer
ENV WORKDIR=/home/developer

# Install oh my zsh & change theme to af-magic
RUN set -eo pipefail && \
    curl -fsSL https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh | sh && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' .zshrc