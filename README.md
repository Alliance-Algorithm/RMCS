# RMCS
RoboMaster Control System based on ROS2.

## Dependency
使用脚本，一键配置开发需要的软件包和插件
```bash
# 拉好镜像，配置好 Devcontainer，进入工作目录
cd /workspaces/rmcs
# 执行脚本
./script/setup.sh
# 构建程序并加载构建结果至当前终端环境中
source ./script/build.sh
```



## Develop

### Some Commands

这些是常见指令
```bash
# 添加ros2的环境变量到当前终端中
source /opt/ros/humble/setup.zsh

# ros2包的依赖自动安装
# cd /path/to/develop_ws/
sudo rosdep install --from-paths src --ignore-src -r -y

# ros2包的构建
# cd /path/to/develop_ws/
colcon build --merge-install

# 加载ros2包的环境变量
# cd /path/to/develop_ws/
source ./install/setup.zsh

# RoboMaster，启动！
ros2 launch rmcs_bringup sentry.yaml
```

### Configure environment

TODO .

1. `ubuntu-server`
2. `ssh-server`
3. `docker`

## Connect with nuc
### LINK-LOCAL (recommend)
#### some useful link
- [network-manager ipv6-addr-gen-mode](https://askubuntu.com/questions/1268900/what-is-setting-my-ipv6-addr-gen-mode)
- [arch linux](https://wiki.archlinuxcn.org/wiki/IPv6)
- [IPv6-link-local-address](https://forums.fedoraforum.org/showthread.php?329517-IPv6-link-local-address)

#### Use NmTui
将`netplan`配置中的`renderer`修改成`NetworkManager`即可使用`nmcli`工具

使用前记得下载：`sudo apt install network-manager`

装系统时会连接网络，下面的配置或许会有wifi的连接配置，保留即可，你也不想让你的设备不能联网吧

```yaml
network:
  renderer: NetworkManager
  ethernets:
    eno1:
      optional: true
      dhcp4: true
  version: 2
  wifis: {}
```

在终端中输入：`nmtui`，后续配置可能会遇上没有权限的情况，使用`sudo`启动即可

```
┌─┤ NetworkManager TUI ├──┐
│                         │
│ Please select an option │
│                         │
│ Edit a connection       │
│ Activate a connection   │
│ Set system hostname     │
│                         │
│ Quit                    │
│                         │
│                    <OK> │
│                         │
└─────────────────────────┘ 
```

进入`Edit a connection`，选择网卡对应的连接，把`IPv6 CONFIGURATION`修改成`<Link-Local>`即可，记得选择 **OK** 哦

接着输入回到终端输入`nmcli`查看对应网卡设备的`local-link`，当然，`ifconfig`，和`ip a`都可以
```text
inet6 fe80::1e83:41ff:fec3:ce3/64
```
这个`fe80`开头的就是，我们用这个链接就可以进行`ssh`，只要两台设备在同一局域网，或者连上了网线就能连接

```text
ssh creeper@fe80::2e0:4cff:fe60:11d5%eno1
            ^^^^^^^^^^^^^^^^^^^^^^^^ ^^^^
              local-link去掉‘/64’   连接用网卡
```

### Net plan (optional)

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
              macaddress: xx:xx:xx:xx:xx:xx

        # 这是需要增加的网卡设备
        enx6c1ff703689c:                    # device name
            optional: true                  # optional
            set-name: xxxxxx                # name
            addresses: [xxx.xxx.xxx.xxx/24] # static ip address + /24
            match:                          # match:
              macaddress: xx:xx:xx:xx:xx:xx # mac address

    wifis:
        wlp2s0:
            optional: true                  # optional enable
            access-points:
                AllianceTeam5.8G:
                    password: rm-alliance.icu
            dhcp4: true
```

### DHCP server (optional)

```sh
# After you set local ip to 192.168.234.1
# And check what is the name of your ethernet device
docker run -it --rm --privileged --network=host qzhhhi/dhcp-server eno1
```

### Sync and Start
 
@todo(creeper) adapt link-local
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
