# 23.0 is the minimum docker engine version that required to build.
# If your docker engine lower than this version, please upgrade it or manually enable the BuildKit.



# Base container, provides a runtime environment
FROM ros:humble AS rmcs-base

# Change bash as default shell instead of sh
SHELL ["/bin/bash", "-c"]

# Set timezone and non-interactive mode
ENV TZ=Asia/Shanghai \
    DEBIAN_FRONTEND=noninteractive

# Modify the apt source
RUN sed -i 's@//.*archive.ubuntu.com@//mirrors.bfsu.edu.cn@g' /etc/apt/sources.list && \
    sed -i 's@//.*security.ubuntu.com@//mirrors.bfsu.edu.cn@g' /etc/apt/sources.list

# Install tools and libraries.
RUN apt-get update && apt-get -y install \
    vim wget curl unzip \
    zsh screen tmux \
    usbutils net-tools iputils-ping \
    libusb-1.0-0-dev \
    libeigen3-dev \
    libopencv-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libceres-dev \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-foxglove-bridge \
    dotnet-sdk-8.0 \
    libpcl-ros-dev libpcl-dev \
    ros-humble-pcl-conversions ros-humble-pcl-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install openvino 2024.6 runtime
RUN wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    apt-key add ./GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    rm ./GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    echo "deb https://apt.repos.intel.com/openvino/2024 ubuntu22 main" > /etc/apt/sources.list.d/intel-openvino-2024.list && \
    apt-get update && \
    apt-get install -y openvino-2024.6.0 && \
    rm -rf /var/lib/apt/lists/*

# Install Livox SDK
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd ./Livox-SDK2/ && \
    mkdir build && cd build && \
    cmake .. && make -j && \
    make install && \
    cd ../.. && rm -rf ./Livox-SDK2/

# Mount rmcs source and install dependencies
RUN --mount=type=bind,target=/tmp/rmcs_ws/src,source=rmcs_ws/src,readonly \
    apt-get update && \
    rosdep install --from-paths /tmp/rmcs_ws/src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Install unison to allow file synchronization
RUN cd /tmp && \
    wget -O unison.tar.gz https://github.com/bcpierce00/unison/releases/download/v2.53.7/unison-2.53.7-ubuntu-x86_64.tar.gz && \
    mkdir -p unison && tar -zxf unison.tar.gz -C unison && \
    cp unison/bin/* /usr/local/bin && \
    rm -rf unison unison.tar.gz



# Developing container, works with devcontainer
FROM rmcs-base AS rmcs-develop

# Install develop tools
RUN apt-get update && apt-get -y install \
    libc6-dev gcc-12 g++-12 \
    cmake make ninja-build \
    openssh-client \
    lsb-release software-properties-common gnupg sudo \
    python3-colorama python3-dpkt && \
    wget -O ./llvm-snapshot.gpg.key https://apt.llvm.org/llvm-snapshot.gpg.key && \
    apt-key add ./llvm-snapshot.gpg.key && \
    rm ./llvm-snapshot.gpg.key && \
    echo "deb https://apt.llvm.org/jammy/ llvm-toolchain-jammy main" > /etc/apt/sources.list.d/llvm-apt.list && \
    apt-get update && \
    version=`apt-cache search clangd- | grep clangd- | awk -F' ' '{print $1}' | sort -V | tail -1 | cut -d- -f2` && \
    apt-get install -y clangd-$version && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 50 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 50 && \
    update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-$version 50 && \
    rm -rf /var/lib/apt/lists/*

# Add user
RUN useradd -m developer --shell /bin/zsh && echo "developer:developer" | chpasswd && adduser developer sudo && \
    echo "developer ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    gpasswd --add developer dialout
WORKDIR /home/developer
ENV USER=developer
ENV WORKDIR=/home/developer

# Generate/load ssh key and setup unison
RUN --mount=type=bind,target=/tmp/.ssh,source=.ssh,readonly=false \
    mkdir -p .ssh && \
    if [ ! -f "/tmp/.ssh/id_rsa" ]; then ssh-keygen -N "" -f "/tmp/.ssh/id_rsa"; fi && \
    cp -r /tmp/.ssh/* .ssh && \
    chown -R 1000:1000 .ssh && chmod 600 .ssh/id_rsa && \
    mkdir -p .unison && \
    echo 'confirmbigdel = false' >> ".unison/default.prf" && \
    chown -R 1000:1000 .unison

USER developer

# Install oh my zsh, change theme to af-magic and setup environment of zsh
RUN sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc && \
    echo 'source ~/env_setup.zsh' >> ~/.zshrc && \
    echo 'export PATH="$PATH:/opt/nvim-linux-x86_64/bin"' >> ~/.zshrc && \
    echo 'export PATH=${PATH}:/workspaces/RMCS/.script' >> ~/.zshrc

# Install latest neovim
RUN curl -LO https://github.com/neovim/neovim/releases/latest/download/nvim-linux-x86_64.tar.gz && \
    sudo rm -rf /opt/nvim && \
    sudo tar -C /opt -xzf nvim-linux-x86_64.tar.gz && \
    rm nvim-linux-x86_64.tar.gz

# Copy environment setup scripts
COPY --chown=1000:1000 .script/template/env_setup.bash env_setup.bash
COPY --chown=1000:1000 .script/template/env_setup.zsh env_setup.zsh

# Runtime container, will automatically launch the main program
FROM rmcs-base AS rmcs-runtime

# Install runtime tools
RUN apt-get update && \
    apt-get install -y tini openssh-server avahi-daemon && \
    rm -rf /var/lib/apt/lists/* && \
    echo 'Port 2022' >> /etc/ssh/sshd_config && \
    echo 'PermitRootLogin yes' >> /etc/ssh/sshd_config && \
    echo 'PasswordAuthentication no' >> /etc/ssh/sshd_config && \
    sed -i 's/#enable-dbus=yes/enable-dbus=no/g' /etc/avahi/avahi-daemon.conf

# Install oh my zsh, disable plugins and update, setup environment and set zsh as default shell
RUN sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" && \
    sed -i 's/plugins=(git)/plugins=()/g' ~/.zshrc && \
    sed -i "s/# zstyle ':omz:update' mode disabled/zstyle ':omz:update' mode disabled/g" ~/.zshrc && \
    echo 'source ~/env_setup.zsh' >> ~/.zshrc && \
    echo 'export PATH=${PATH}:/rmcs_install/lib/rmcs_cli' >> ~/.zshrc && \
    chsh -s /bin/zsh root

RUN mkdir -p /rmcs_install/

COPY --chown=root:root .script/set-robot /usr/local/bin/set-robot
COPY --chown=root:root .script/template/set-hostname /usr/local/bin/set-hostname

COPY --chown=root:root .script/template/entrypoint /entrypoint
COPY --chown=root:root .script/template/rmcs-service /etc/init.d/rmcs

COPY --from=rmcs-develop --chown=root:root /home/developer/.ssh/id_rsa.pub /root/.ssh/authorized_keys

WORKDIR /root/
COPY --chown=root:root .script/template/env_setup.bash env_setup.bash
COPY --chown=root:root .script/template/env_setup.zsh env_setup.zsh

ENTRYPOINT ["tini", "--"]
CMD [ "/entrypoint" ]
