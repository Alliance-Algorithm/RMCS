# Runtime container
FROM ros:humble AS rmcs-runtime

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

# Download RMCS source, fix dependencies and compile
RUN git clone https://github.com/Alliance-Algorithm/RMCS && \
    cd RMCS && git switch motor_test && cd rmcs_ws && \
    source /opt/ros/humble/local_setup.bash && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/* && \
    colcon build --symlink-install && \
    mv ./install /rmcs-runtime && cd / && rm -rf RMCS

# Developing container
FROM rmcs-runtime AS rmcs-develop

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