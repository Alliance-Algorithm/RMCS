# RMCS
RoboMaster Control System based on ROS2.

## Dependency

```zsh
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc

# if you keep the origin folder structure
cd /workspaces/RMCS/rmcs_ws

# build to make custom packages available
colcon build --merge-install

sudo rosdep install --from-paths src --ignore-src -r -y
```

## Develop

#### Configure environment

TODO .

1. `ubuntu-server`
2. `ssh-server`
3. `docker`

### Connect with nuc
#### Net plan

```yaml
# in /etc/netplan/****.yaml
network:
    renderer: networkd
    version: 2
    ethernets:
        enp1s0:
            optional: true                  # optional enable
            set-name: mid-360
            addresses: [192.168.1.50/24]
            match:
              macaddress: 70:70:fc:03:7f:83

        enx6c1ff703689c:                    # device name
            optional: true                  # optional
            set-name: qzhhhi                # name
            addresses: [192.168.234.2/24]   # static ip address + /24
            match:                          # match:
              macaddress: 6c:1f:f7:03:68:9c # mac address

        enx207bd27efaf5:
            optional: true
            set-name: creeper5820
            addresses: [192.168.234.3/24]
            match:  
              macaddress: 20:7b:d2:7e:fa:f5

    wifis:
        wlp2s0:
            optional: true                  # optional enable
            access-points:
                AllianceTeam5.8G:
                    password: rm-alliance.icu
            dhcp4: true
```

#### DHCP server (optional)

```sh
# After you set local ip to 192.168.234.1
# And check what is the name of your ethernet device
docker run -it --rm --privileged --network=host qzhhhi/dhcp-server eno1
```

#### Sync and Start
 
```sh
# Set remote ip
set-remote 192.168.234.2

# ssh
ssh-remote

# Copy the install directory to remote
# TODO: Remove directory restrictions
sync-remote
```

## How to make NUC connect to Internet

### Gateway

To make Ubuntu1 regarded as gateway, edit the `/etc/netplan/***.yaml`. A possible version is shown below.

```
# Let NetworkManager manage all devices on this system
network:
    ethernets:
        ens32:                    ## network card name
            dhcp4: false
            addresses:
              - 192.168.234.5/24   ## set static IP
            routes:
              - to: default
                via: 192.168.234.1  ## gateway
    version: 2
```

### Soft Route

Assuming that Ubuntu 1 has wlp4s0 connected to Internet and eno1 connected to NUC, Run commands below in Ubuntu 1 to create a soft route.

``` bash

sudo iptables -A FORWARD -i eno1 -o wlp4s0 -j ACCEPT
sudo iptables -A FORWARD -i wlp4s0 -o eno1 -m state --state ESTABLISHED,RELATED -j ACCEPT
sudo iptables -t nat -A POSTROUTING -o wlp4s0 -j MASQUERADE


```

## Remap Port of Omni Direction Perception Camera

1. Check out Pid and Vid.

``` bash
lsusb | grep DECXIN
```

As shown below, Pid is 2cd1 while Vid is 1bcf.

> Bus 003 Device 011: ID 1bcf:2cd1 Sunplus Innovation Technology Inc. DECXIN  CAMERA

2. Check out port.

``` bash
udevadm info --attribute-walk --name=/dev/video2 | grep "KERNEL"
```

As shown below, port is "3-2".

> KERNEL=="video2" \
  KERNELS=="3-2:1.0" \
  KERNELS=="3-2" \
  KERNELS=="usb3" \
  KERNELS=="0000:36:00.4" \
  KERNELS=="0000:00:08.1" \
  KERNELS=="pci0000:00" 

3. Edit udev rules

``` bash
sudo gedit /etc/udev/rules.d/omni.rules 
```

Type rules below
```
KERNEL=="video*",KERNELS=="3-2", ATTRS{idProduct}=="2cd1", ATTRS{idVendor}=="1bcf", MODE:="0777", SYMLINK+="leftfront"
KERNEL=="video*",KERNELS=="3-1", ATTRS{idProduct}=="2cd1", ATTRS{idVendor}=="1bcf", MODE:="0777", SYMLINK+="rightfront"
KERNEL=="video*",KERNELS=="3-3", ATTRS{idProduct}=="2cd1", ATTRS{idVendor}=="1bcf", MODE:="0777", SYMLINK+="left"
KERNEL=="video*",KERNELS=="3-4", ATTRS{idProduct}=="2cd1", ATTRS{idVendor}=="1bcf", MODE:="0777", SYMLINK+="right"
```

4. Refresh

``` bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Runtime environment build

1. Pull or load images

- Online

``` bash
sudo docker pull qzhhhi/rmcs-runtime 
```

- Offline 
``` bash
sudo docker load -i rmcs-runtime.tar
```

2. Start with NUC

```
sudo vi /etc/init.d/rmcs-runtime.sh
```

Type content below

``` bash
### BEGIN INIT INFO
# Providers:         rmcs-runtime.sh
# Description:       run rmcs runtime environment
### END INIT INFO
 
 
systemctl start docker
docker run --restart=always --privileged --network=host -v /dev:/dev qzhhhi/rmcs-runtime
```

> PS: `### BEGIN INIT INFO` and `### END INIT INFO` are required.

```bash
sudo chmod 775 rmcs-runtime.sh
sudo update-rc.d rmcs-runtime.sh defaults 90 
```

## Gimbal calibration

``` bash
source /opt/ros/humble/setup.bash 
ros2 topic pub -1 /gimbal/calibrate std_msgs/msg/Int32 "{'data':1}"
cat /rmcs.launch.out
```

## Gyro calibration

``` bash
source /opt/ros/humble/setup.bash 
ros2 topic pub -1 /imu/calibrator std_msgs/msg/Int8 "{'data':1}" # Start calibrating
ros2 topic pub -1 /imu/calibrator std_msgs/msg/Int8 "{'data':0}" # Finish calibrating
cat /rmcs.launch.out
```

> PS: All control should be disabled when calibrating.
