# 23.0 is the minimum docker engine version that required to build.
# If your docker engine lower than this version, please upgrade it or manually enable the BuildKit.



# Base container, provides a runtime environment
FROM ros:humble AS rmcs-base

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

# Mount rmcs source, fix dependencies and compile
RUN --mount=type=bind,target=/tmp/rmcs_ws/src,source=rmcs_ws/src,readonly \
    cd /tmp/rmcs_ws && \
    source /opt/ros/humble/local_setup.bash && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/* && \
    colcon build --install-base /rmcs_install --merge-install && \
    rm -rf build log

# Install unison to allow file synchronization
RUN cd /tmp && \
    wget -O unison.tar.gz https://github.com/bcpierce00/unison/releases/download/v2.53.4/unison-2.53.4-ubuntu-x86_64.tar.gz && \
    mkdir -p unison && tar -zxf unison.tar.gz -C unison && \
    cp unison/bin/* /usr/local/bin && \
    rm -rf unison unison.tar.gz



# Developing container, works with devcontainer
FROM rmcs-base AS rmcs-develop

# Install develop tools (ssh/clangd/zsh/etc...)
RUN apt-get update && apt-get -y install \
    openssh-client \
    lsb-release software-properties-common gnupg zsh sudo \
    libcanberra-gtk-module libcanberra-gtk3-module && \
    rm -rf /var/lib/apt/lists/* \
    echo -e "\n" | bash -c "$(wget -O - https://apt.llvm.org/llvm.sh)" && \
    ln -s /usr/bin/clangd-* /usr/bin/clangd

# Add user
RUN useradd -m developer --shell /bin/zsh && echo "developer:developer" | chpasswd && adduser developer sudo && \
    echo "developer ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    gpasswd --add developer dialout
WORKDIR /home/developer
ENV USER=developer
ENV WORKDIR=/home/developer

# Generate ssh secret key & profile unison
RUN --mount=type=bind,target=/tmp/.ssh,source=.ssh,readonly \
    mkdir -p .ssh && \
    if [ ! -f "/tmp/.ssh/id_rsa.pub" ]; then ssh-keygen -N "" -f ".ssh/id_rsa"; fi && \
    cp -r /tmp/.ssh/* .ssh && \
    mkdir -p .unison && \
    echo 'confirmbigdel = false' >> ".unison/default.prf" && \
    chown -R 1000:1000 .ssh .unison

USER developer

# Install oh my zsh & change theme to af-magic
RUN set -eo pipefail && \
    curl -fsSL https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh | sh && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' .zshrc

COPY --chown=root:root .script/set-remote.py /usr/local/bin/set-remote
COPY --chown=root:root .script/ssh-remote.sh /usr/local/bin/ssh-remote
COPY --chown=root:root .script/sync-remote.sh /usr/local/bin/sync-remote
COPY --chown=root:root .script/service-remote.sh /usr/local/bin/service-remote



# Runtime container, will automatically launch the main program
FROM rmcs-base AS rmcs-runtime

# Install ssh server
RUN apt-get update && \
    apt-get install -y openssh-server && \
    rm -rf /var/lib/apt/lists/* && \
    echo 'Port 2022' >> /etc/ssh/sshd_config && \
    echo 'PermitRootLogin yes' >> /etc/ssh/sshd_config && \
    echo 'PasswordAuthentication no' >> /etc/ssh/sshd_config

# Add tini
RUN wget -O /tini https://github.com/krallin/tini/releases/download/v0.19.0/tini && \
    chmod +x /tini

# Entrypoint
RUN echo '#!/usr/bin/bash' > /entrypoint.sh && \
    echo 'rm -rf /tmp/*' >> /entrypoint.sh && \
    echo 'service ssh restart' >> /entrypoint.sh && \
    echo 'service rmcs restart' >> /entrypoint.sh && \
    echo 'sleep infinity' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

COPY --chown=root:root .script/service.sh /etc/init.d/rmcs
COPY --from=rmcs-develop --chmod=600 --chown=root:root /home/developer/.ssh/id_rsa.pub /root/.ssh/authorized_keys

WORKDIR /root/

ENTRYPOINT ["/tini", "--"]
CMD [ "/entrypoint.sh" ]