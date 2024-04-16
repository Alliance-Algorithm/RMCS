# RMCS
RoboMaster Control System based on ROS2.

## Develop

#### Configure environment

TODO .

1. `ubuntu-server`
1. `ssh-server`
1. `docker`

#### Docker without sudo
```sh
sudo groupadd docker
sudo gpasswd -a ${USER} docker
sudo systemctl restart docekr
sudo chmod a+rw /var/run/docker.sock
```

#### DHCP server

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
