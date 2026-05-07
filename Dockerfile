# 23.0 is the minimum docker engine version that required to build.
# If your docker engine lower than this version, please upgrade it or manually enable the BuildKit.



# Sysroot source images for rmcs-develop-full.
# CI overrides these with digest-pinned references. For local builds, build
# rmcs-base for both architectures first and pass --build-arg explicitly;
# see docs/zh-cn/build_docker_image.md.
ARG SYSROOT_IMAGE_AMD64=rmcs-base:latest
ARG SYSROOT_IMAGE_ARM64=rmcs-base:latest

# Base container, provides a runtime environment
FROM ros:jazzy AS rmcs-base
ARG TARGETARCH

# Change bash as default shell instead of sh
SHELL ["/bin/bash", "-c"]

# Set timezone and non-interactive mode
ENV TZ=Asia/Shanghai \
    DEBIAN_FRONTEND=noninteractive

# Install tools and libraries.
RUN apt-get update && apt-get install -y --no-install-recommends \
    vim wget curl unzip \
    zsh screen tmux \
    usbutils net-tools iputils-ping \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    ripgrep htop fzf \
    libusb-1.0-0-dev \
    libeigen3-dev \
    libopencv-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libceres-dev \
    ros-$ROS_DISTRO-rviz2 ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pcl-conversions ros-$ROS_DISTRO-pcl-msgs \
    ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-msgs \
    lua5.4 liblua5.4-0 liblua5.4-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# Install Hik MVS runtime SDK and its runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libbz2-1.0 \
    libgcc-s1 \
    libstdc++6 \
    libudev1 \
    libusb-1.0-0 \
    zlib1g && \
    apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* && \
    set -eux; \
    case "${TARGETARCH}" in \
        amd64) arch_tag=x86_64; arch_dir=64 ;; \
        arm64) arch_tag=aarch64; arch_dir=aarch64 ;; \
        *) echo "unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac; \
    download_url="https://github.com/Alliance-Algorithm/hik-mvs/releases/latest/download/mvs-sdk-${arch_tag}.tar.gz"; \
    mkdir -p /tmp/mvs-src \
        "/opt/mvs-usb3-core/lib/${arch_dir}" \
        /opt/mvs-usb3-core/lib/cmake/MVSUSB3Core \
        /opt/mvs-usb3-core/include \
        /usr/local/bin; \
    curl -fsSL -o /tmp/mvs.tar.gz "${download_url}"; \
    tar -xzf /tmp/mvs.tar.gz -C /tmp/mvs-src; \
    cp -a "/tmp/mvs-src/lib/${arch_dir}/." "/opt/mvs-usb3-core/lib/${arch_dir}/"; \
    cp -a /tmp/mvs-src/include/. /opt/mvs-usb3-core/include/; \
    rm -f "/opt/mvs-usb3-core/lib/${arch_dir}/libusb-1.0.so.0"; \
    case "${TARGETARCH}" in \
        amd64) \
            rm -f \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libCLAllSerial_gcc485_v3_0.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libCLProtocol_gcc485_v3_0.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libCLSerCOM.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libCLSerHvc.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libGCBase_gcc485_v3_0.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libGenCP_gcc485_v3_0.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/liblog4cpp_gcc485_v3_0.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libLog_gcc485_v3_0.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libMvCameraControlWrapper.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libMvCameraControlWrapper.so.4.7.0.1" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libMvCamLVision.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libMvCamLVision.so.4.7.0.3" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libMVGigEVisionSDK.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libMVGigEVisionSDK.so.4.7.1.1" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libMVFGControl.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libMvProducerVIR.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libMvLCProducer.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/MvLCProducer.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/MvProducerGEV.cti" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/MvFGProducerCML.cti" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/MvFGProducerCXP.cti" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/MvFGProducerGEV.cti" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/MvFGProducerXoF.cti"; \
            ;; \
        arm64) \
            rm -f \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libCLAllSerial_gcc494_v3_0.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libCLProtocol_gcc494_v3_0.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libCLSerCOM.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libGCBase_gcc494_v3_0.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libGenCP_gcc494_v3_0.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/liblog4cpp_gcc494_v3_0.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libLog_gcc494_v3_0.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libMvCameraControlWrapper.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libMvCameraControlWrapper.so.4.7.0.1" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libMvCamLVision.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libMvCamLVision.so.4.7.0.3" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libMVGigEVisionSDK.so" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/libMVGigEVisionSDK.so.4.7.1.1" \
                "/opt/mvs-usb3-core/lib/${arch_dir}/MvProducerGEV.cti"; \
            ;; \
        *) echo "unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac; \
    rm -rf /var/lib/apt/lists/* /tmp/mvs-src /tmp/mvs.tar.gz; \
    cmake_dir=/opt/mvs-usb3-core/lib/cmake/MVSUSB3Core; \
    <<EOF cat > "${cmake_dir}/MVSUSB3CoreConfig.cmake"
get_filename_component(MVSUSB3Core_ROOT "\${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

set(MVSUSB3Core_INCLUDE_DIR "\${MVSUSB3Core_ROOT}/include")
set(MVSUSB3Core_INCLUDE_DIRS "\${MVSUSB3Core_INCLUDE_DIR}")
set(MVSUSB3Core_LIBRARY_DIR "\${MVSUSB3Core_ROOT}/lib/${arch_dir}")
set(MVSUSB3Core_LIBRARY "\${MVSUSB3Core_LIBRARY_DIR}/libMvCameraControl.so")
set(MVSUSB3Core_LIBRARIES "\${MVSUSB3Core_LIBRARY}")

if(NOT EXISTS "\${MVSUSB3Core_INCLUDE_DIR}/MvCameraControl.h")
  set(MVSUSB3Core_FOUND FALSE)
  message(FATAL_ERROR "MVSUSB3Core headers not found under \${MVSUSB3Core_INCLUDE_DIR}")
endif()

if(NOT EXISTS "\${MVSUSB3Core_LIBRARY}")
  set(MVSUSB3Core_FOUND FALSE)
  message(FATAL_ERROR "MVSUSB3Core library libMvCameraControl.so not found under \${MVSUSB3Core_LIBRARY_DIR}")
endif()

if(NOT TARGET MVSUSB3Core::MVSUSB3Core)
  add_library(MVSUSB3Core::MVSUSB3Core SHARED IMPORTED GLOBAL)
  set_target_properties(MVSUSB3Core::MVSUSB3Core PROPERTIES
    IMPORTED_LOCATION "\${MVSUSB3Core_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "\${MVSUSB3Core_INCLUDE_DIR}"
  )
endif()

set(MVSUSB3Core_FOUND TRUE)
EOF


# Install openvino runtime
RUN if [ "${TARGETARCH}" = "amd64" ]; then \
        mkdir -p /etc/apt/keyrings && \
        wget -O /etc/apt/keyrings/intel-openvino.asc \
            https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
        chmod 644 /etc/apt/keyrings/intel-openvino.asc && \
        echo "deb [signed-by=/etc/apt/keyrings/intel-openvino.asc] https://apt.repos.intel.com/openvino ubuntu24 main" \
            > /etc/apt/sources.list.d/intel-openvino.list && \
        apt-get update && \
        apt-get install -y --no-install-recommends openvino-2025.2.0 && \
        apt-get clean && \
        rm -rf /var/lib/apt/lists/* /tmp/*; \
    else \
        echo "Skipping OpenVINO install on ${TARGETARCH}"; \
    fi

# Install Livox SDK
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && \
    sed -i '6iset(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-pragmas -Wno-c++20-compat -include cstdint")' CMakeLists.txt && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j && \
    make install && \
    cd ../.. && rm -rf Livox-SDK2

# Mount rmcs source and install dependencies
RUN --mount=type=bind,target=/rmcs_ws/src,source=rmcs_ws/src,readonly \
    apt-get update && \
    rosdep install --from-paths /rmcs_ws/src --ignore-src -r -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# Build Unison from source because upstream does not ship arm64 release binaries
# and the distro package does not include unison-fsmonitor.
# SHA256 verified against Ubuntu Launchpad source package unison-2.53.8-1.
RUN export UNISON_VERSION=2.53.8 && \
    apt-get update && \
    apt-get install -y --no-install-recommends ocaml-nox && \
    curl -L "https://github.com/bcpierce00/unison/archive/refs/tags/v${UNISON_VERSION}.tar.gz" \
        -o "/tmp/unison-v${UNISON_VERSION}.tar.gz" && \
    echo "d0d30ea63e09fc8edf10bd8cbab238fffc8ed510d27741d06b5caa816abd58b6  /tmp/unison-v${UNISON_VERSION}.tar.gz" | sha256sum -c - && \
    tar -xzf "/tmp/unison-v${UNISON_VERSION}.tar.gz" -C /tmp && \
    cd "/tmp/unison-${UNISON_VERSION}" && \
    make -j"$(nproc)" && \
    make install PREFIX=/usr/local && \
    cd / && \
    rm -rf "/tmp/unison-${UNISON_VERSION}" "/tmp/unison-v${UNISON_VERSION}.tar.gz" && \
    apt-get purge -y --auto-remove ocaml-nox && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# Developing container, works with devcontainer
FROM rmcs-base AS rmcs-develop
ARG TARGETARCH

# Install develop tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    libc6-dev gcc-14 g++-14 \
    cmake make ninja-build \
    openssh-client \
    lsb-release software-properties-common gnupg sudo \
    python3-colorama python3-dpkt && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 50 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-14 50 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# Install Node.js 24 LTS (required by agent CLIs)
RUN curl -fsSL https://deb.nodesource.com/setup_24.x | bash - && \
    apt-get install -y --no-install-recommends nodejs && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# Install llvm-toolchain
ARG LLVM_VERSION=22
RUN mkdir -p /etc/apt/keyrings && \
    wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key | gpg --dearmor -o /etc/apt/keyrings/apt.llvm.org.gpg && \
    chmod 644 /etc/apt/keyrings/apt.llvm.org.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/apt.llvm.org.gpg] https://apt.llvm.org/noble/ llvm-toolchain-noble-${LLVM_VERSION} main" \
    > /etc/apt/sources.list.d/llvm.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    libomp-${LLVM_VERSION}-dev \
    clang-${LLVM_VERSION} clangd-${LLVM_VERSION} clang-format-${LLVM_VERSION} clang-tidy-${LLVM_VERSION} \
    lldb-${LLVM_VERSION} lld-${LLVM_VERSION} llvm-${LLVM_VERSION} && \
    update-alternatives --install /usr/bin/clang clang /usr/bin/clang-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/lldb lldb /usr/bin/lldb-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/llvm-ar llvm-ar /usr/bin/llvm-ar-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/llvm-ranlib llvm-ranlib /usr/bin/llvm-ranlib-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/ld.lld ld.lld /usr/bin/ld.lld-${LLVM_VERSION} 50 && \
    apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/*

# Generate/load ssh key and setup unison
RUN --mount=type=bind,target=/tmp/.ssh,source=.ssh,readonly=false \
    cd /home/ubuntu && mkdir -p .ssh && \
    if [ ! -f "/tmp/.ssh/id_rsa" ]; then ssh-keygen -N "" -f "/tmp/.ssh/id_rsa"; fi && \
    cp -r /tmp/.ssh/* .ssh && \
    chown -R 1000:1000 .ssh && chmod 600 .ssh/id_rsa && \
    mkdir -p .unison && \
    echo 'confirmbigdel = false' >> ".unison/default.prf" && \
    chown -R 1000:1000 .unison

# Install latest neovim
RUN case "${TARGETARCH}" in \
        amd64) nvim_arch=x86_64 ;; \
        arm64) nvim_arch=arm64 ;; \
        *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac && \
    curl -LO https://github.com/neovim/neovim/releases/latest/download/nvim-linux-${nvim_arch}.tar.gz && \
    rm -rf /opt/nvim && \
    tar -C /opt -xzf nvim-linux-${nvim_arch}.tar.gz && \
    mv /opt/nvim-linux-${nvim_arch} /opt/nvim && \
    rm nvim-linux-${nvim_arch}.tar.gz
ENV PATH="${PATH}:/opt/nvim/bin"

# Install latest stable cmake for user ubuntu
RUN case "${TARGETARCH}" in \
        amd64) cmake_arch=x86_64 ;; \
        arm64) cmake_arch=aarch64 ;; \
        *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac && \
    wget https://github.com/kitware/cmake/releases/download/v4.2.3/cmake-4.2.3-linux-${cmake_arch}.sh -O install.sh && \
    mkdir -p /opt/cmake/ && bash install.sh --skip-license --prefix=/opt/cmake/ --exclude-subdir && \
    rm install.sh

# Change user
RUN chsh -s /bin/zsh ubuntu && \
    echo "ubuntu ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers

# Precreate generic XDG-style parent directories for direct bind mounts under ubuntu's home.
RUN mkdir -p \
        /home/ubuntu/.agents \
        /home/ubuntu/.cache \
        /home/ubuntu/.config \
        /home/ubuntu/.local/share \
        /home/ubuntu/.local/state && \
    chown -R ubuntu:ubuntu /home/ubuntu/.agents /home/ubuntu/.cache /home/ubuntu/.config /home/ubuntu/.local
WORKDIR /home/ubuntu
ENV USER=ubuntu
ENV WORKDIR=/home/ubuntu
USER ubuntu

# Install oh my zsh, change theme to af-magic and setup environment of zsh
RUN sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc && \
    echo '# Hint: uncomment and set RMCS_PATH if RMCS is not located at /workspaces/RMCS.' >> ~/.zshrc && \
    echo '# export RMCS_PATH="/workspaces/RMCS"' >> ~/.zshrc && \
    echo 'source ~/env_setup.zsh' >> ~/.zshrc

# Copy environment setup scripts
COPY --chown=1000:1000 .script/template/env_setup.bash env_setup.bash
COPY --chown=1000:1000 .script/template/env_setup.zsh env_setup.zsh

FROM --platform=linux/amd64 ${SYSROOT_IMAGE_AMD64} AS rmcs-sysroot-amd64
FROM --platform=linux/arm64 ${SYSROOT_IMAGE_ARM64} AS rmcs-sysroot-arm64

# Developing container with cross toolchains and opposite-arch sysroot.
FROM rmcs-develop AS rmcs-develop-full
ARG TARGETARCH

USER root

RUN apt-get update && \
    case "${TARGETARCH}" in \
        amd64) cross_triplet=aarch64-linux-gnu; cross_pkg_triplet=aarch64-linux-gnu ;; \
        arm64) cross_triplet=x86_64-linux-gnu; cross_pkg_triplet=x86-64-linux-gnu ;; \
        *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac && \
    apt-get install -y --no-install-recommends \
        "gcc-14-${cross_pkg_triplet}" "g++-14-${cross_pkg_triplet}" \
        "binutils-${cross_pkg_triplet}" && \
    update-alternatives --install "/usr/bin/${cross_triplet}-gcc" "${cross_triplet}-gcc" "/usr/bin/${cross_triplet}-gcc-14" 50 && \
    update-alternatives --install "/usr/bin/${cross_triplet}-g++" "${cross_triplet}-g++" "/usr/bin/${cross_triplet}-g++-14" 50 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

RUN mkdir -p /opt/sysroots && \
    case "${TARGETARCH}" in \
        amd64) mkdir -p /opt/sysroots/arm64 ;; \
        arm64) mkdir -p /opt/sysroots/amd64 ;; \
        *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac

RUN --mount=from=rmcs-sysroot-amd64,target=/mnt/sysroot-amd64,readonly \
    --mount=from=rmcs-sysroot-arm64,target=/mnt/sysroot-arm64,readonly \
    set -euo pipefail && \
    case "${TARGETARCH}" in \
        amd64) tar \
            --exclude='./dev/*' \
            --exclude='./proc/*' \
            --exclude='./sys/*' \
            --exclude='./run/*' \
            --exclude='./tmp/*' \
            -C /mnt/sysroot-arm64 -cf - . | tar -C /opt/sysroots/arm64 -xf - ;; \
        arm64) tar \
            --exclude='./dev/*' \
            --exclude='./proc/*' \
            --exclude='./sys/*' \
            --exclude='./run/*' \
            --exclude='./tmp/*' \
            -C /mnt/sysroot-amd64 -cf - . | tar -C /opt/sysroots/amd64 -xf - ;; \
        *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac

WORKDIR /home/ubuntu
ENV USER=ubuntu
ENV WORKDIR=/home/ubuntu
USER ubuntu


# Runtime container, will automatically launch the main program
FROM rmcs-base AS rmcs-runtime

# Install runtime tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends tini openssh-server avahi-daemon orphan-sysvinit-scripts && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* && \
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

COPY --from=rmcs-develop --chown=root:root /home/ubuntu/.ssh/id_rsa.pub /root/.ssh/authorized_keys

WORKDIR /root/
COPY --chown=root:root .script/template/env_setup.bash env_setup.bash
COPY --chown=root:root .script/template/env_setup.zsh env_setup.zsh

ENTRYPOINT ["tini", "--"]
CMD [ "/entrypoint" ]
