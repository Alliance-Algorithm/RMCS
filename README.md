# RMCS
RoboMaster Control System based on ROS2.

## Develop

#### Configure environment

TODO .

1. `ubuntu-server`
1. `ssh-server`
1. `docker`

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