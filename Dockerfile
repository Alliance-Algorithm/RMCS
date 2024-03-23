# Base container, provides a runtime environment
FROM ros:humble AS rmcs-base

WORKDIR /

# Change bash as default shell instead of sh
SHELL ["/bin/bash", "-c"]

# Install some tools and libraries.
RUN apt-get update && apt-get -y install \
    vim wget curl unzip \
    build-essential \
    cmake \
    make ninja-build \
    libeigen3-dev \
    libopencv-dev \
    libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libsuitesparse-dev \
    libceres-dev \
    && rm -rf /var/lib/apt/lists/*

# Install openvino 2023.3 runtime (C++ API only)
RUN wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    echo "deb https://apt.repos.intel.com/openvino/2023 ubuntu22 main" | tee /etc/apt/sources.list.d/intel-openvino-2023.list && \
    apt-get update && \
    apt-get install -y openvino-2023.3.0 && \
    rm -rf /var/lib/apt/lists/* && \
    rm ./GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB

# Download RMCS source, fix dependencies and compile
RUN git clone --recurse-submodules https://github.com/Alliance-Algorithm/RMCS && \
    cd RMCS/rmcs_ws && \
    source /etc/profile && \
    source /opt/ros/humble/local_setup.bash && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/* && \
    colcon build && \
    mv ./install /rmcs-install && cd / && \
    rm -rf RMCS


# Runtime container, will automatically launch the main program
FROM rmcs-base AS rmcs-runtime

RUN echo '#!/usr/bin/bash' > /entrypoint.bash && \
    echo 'source /opt/ros/humble/setup.bash' >> /entrypoint.bash && \
    echo 'source /rmcs-install/setup.bash' >> /entrypoint.bash && \
    echo 'ros2 launch rmcs_bringup rmcs.launch.py' >> /entrypoint.bash && \
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
    echo "developer ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    gpasswd --add developer dialout

# Set user
USER developer
WORKDIR /home/developer
ENV USER=developer
ENV WORKDIR=/home/developer

# Install oh my zsh & change theme to af-magic
RUN set -eo pipefail && \
    curl -fsSL https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh | sh && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' .zshrc