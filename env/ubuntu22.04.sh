#!/usr/bin/bash

sudo apt-get update
sudo apt-get -y install gnupg wget gpg curl software-properties-common

sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor | sudo tee /usr/share/keyrings/ros2-latest-archive-keyring.gpg > /dev/null
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB

sudo gpg --output /etc/apt/trusted.gpg.d/intel.gpg --dearmor GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
rm GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB

echo "deb https://apt.repos.intel.com/openvino ubuntu22 main" | sudo tee /etc/apt/sources.list.d/intel-openvino.list
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys BAC6F0C353D04109
sudo apt-get update
sudo apt-get install -y libtbb-dev  libeigen3-dev libopencv-dev openvino gcc-13 g++-13 libceres-dev libdwarf-dev libbackward-cpp-dev  binutils-dev libdw-dev  libunwind-dev libfmt-dev libspdlog-dev libyaml-cpp-dev

sudo ln -s /usr/include/libdwarf/libdwarf.h /usr/include/libdwarf.h 
sudo ln -s /usr/include/libdwarf/dwarf.h /usr/include/dwarf.h

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 13
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 13 
sudo update-alternatives --set g++ /usr/bin/g++-13
sudo update-alternatives --set gcc /usr/bin/gcc-13
