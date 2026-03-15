# Prerequisite

To build/develop the flight's branch, you have to do the following tasks as they're not included either in the image nor in package managers.

## Dependencies for Uxrce DDS Client

The `micro-xrce-dds-agent` requires Fast-DDS to be installed.
That package is not included in the official APT package source, so we have to compile it manually.

First, install ASIO.

```bash
sudo apt-get install libasio-dev
```

Then, compile the library from source.

```bash
mkdir -p /tmp/fastdds
git clone https://github.com/eProsima/Fast-CDR.git
mkdir Fast-CDR/build && cd Fast-CDR/build
cmake .. -DCMAKE_INSTALL_PREFIX=/workspaces/RMCS/rmcs_ws/install -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON
cmake --build . --target install -j
cd /tmp/fastdds && git clone https://github.com/eProsima/Fast-DDS.git
mkdir Fast-DDS/build && cd Fast-DDS/build
cmake .. -DCMAKE_INSTALL_PREFIX=/workspaces/RMCS/rmcs_ws/install -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON
cmake --build . --target install -j
cd / && rm -rf /tmp/fastdds
```

This would roughly take ~50s to compile for a 9950x cpu using 16 threads.

And then compile the agent:

```bash
build-rmcs \
    --packages-skip microxrcedds_agent \
    --cmake-clean-cache \
    --cmake-force-configure

build-rmcs \
    --packages-select microxrcedds_agent \
    --cmake-clean-cache \
    --cmake-force-configure \
    --cmake-args \
      '-DBUILD_SHARED_LIBS=OFF' \
      '-DUAGENT_BUILD_EXECUTABLE=ON' \
      '-DUAGENT_USE_SYSTEM_FASTCDR=ON' \
      '-DUAGENT_USE_SYSTEM_FASTDDS=ON' \
      '-Dfastcdr_SHARED_LIBS=OFF' \
      '-Dfastdds_SHARED_LIBS=OFF' \
      '-Dfastcdr_DIR=/workspaces/RMCS/rmcs_ws/install/lib/cmake/fastcdr' \
      '-Dfastdds_DIR=/workspaces/RMCS/rmcs_ws/install/share/fastdds/cmake' \
      '-DCMAKE_PREFIX_PATH:PATH=/workspaces/RMCS/rmcs_ws/install;/opt/ros/jazzy'
```

This would roughly take ~30s to compile for a 9950x cpu using 16 threads.

To run the uxrce dds client:

```bash
sudo MicroXRCEAgent serial --dev /dev/ttyS0 -b 921600
```

Note that the baudrate should be the same as the one configured in the flight controller.

## Dependencies for Odin1 Driver

Odin1 driver requires cv-bridge which is no longer used by the main upstream.

```bash
sudo apt-get install ros-jazzy-cv-bridge
```

Note that you might have to modify Odin1's source code to compile it.
Some of the code refer to `<cv_bridge/cv_bridge.h>` instead of `.hpp`.
