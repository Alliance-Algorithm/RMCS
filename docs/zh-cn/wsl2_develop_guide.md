# WSL2 Development Guide

由于 **Windows 版本** 会显著影响在 WSL2 下的开发体验（例如镜像模式网络无法在低版本中使用），本指南假定您拥有一台运行 **Windows 11 22H2** 或更高版本的设备。

> 检查Windows版本：按 Win + R 输入 `winver` 回车即可显示。

**对于较低版本的 Windows 使用者，建议安装双系统进行开发。**

如果您有在其它版本的 Windows 中优雅的使用 WSL 开发 RMCS 的诀窍，可以向我们提出 Issue 或 Pull Request。

##

### Step 1
在设置 `系统 > 开发者选项` 中，找到 `启用 sudo` 选项，将其开启。

### Step 2

在 Windows 的用户目录（通常为 `C:\Users\<your-username>`）中，新建文件 `.wslconfig`，填充以下内容：

```ini
[experimental]
networkingMode=mirrored
dnsTunneling=true
firewall=true
autoProxy=true
```

配置此设置以启用镜像模式网络。

> 在运行 Windows 11 22H2 及更高版本的计算机上，可以启用 [**镜像模式网络**](https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking)。
> 
> 启用此功能会将 WSL 更改为全新的网络体系结构，其目标是将 Windows 上的网络接口“镜像”到 Linux 中，以添加新的网络功能并提高兼容性。

### Step 3

若 WSL 未安装，则打开 Windows 终端，输入

```powershell
sudo wsl --install
```

此命令将启用运行最新版 WSL2，并安装 Ubuntu 发行版。

若 WSL 已安装，则输入

```powershell
wsl --shutdown
```

和

```powershell
wsl
```

重启 WSL 以应用更改。

### Step 4

[安装 docker 并配置代理（部分国家或地区）](docker_with_proxy.md)。

不建议安装 Docker Desktop。

### Step 5 (Optional)

在 Windows 终端中输入：

```powershell
winget install --interactive --exact dorssel.usbipd-win
```

这将安装（过程可能需要重启）：

- 名为 usbipd 的服务（显示名称：USBIP 设备主机）。 可使用 Windows 中的“服务”应用检查此服务的状态。
- 命令行工具 usbipd。 此工具的位置将添加到 PATH 环境变量。
- 名为 usbipd 的防火墙规则，用于允许所有本地子网连接到服务。 可修改此防火墙规则以微调访问控制。

完成后，将 [下位机](https://github.com/Alliance-Algorithm/rmcs_slave) 接入PC，在终端中输入：

```powershell
usbipd list
```

如果一切正常，你将得到类似如下的回应：
```
Connected:
BUSID  VID:PID    DEVICE              STATE
1-3    a11c:75f3  RMCS Slave v2.1.2   Not shared
...
```

复制显示的设备 BUSID（示例里为 `1-3`），使用命令共享设备（修改 `<busid>` 为复制的值）：

```powershell
sudo usbipd bind --busid <busid>
```

再次使用命令 `usbipd list` 检查设备状态，可以发现其 `STATE` 栏的内容变为 `Shared` 。

接下来使用命令将设备附加到 WSL：

```powershell
usbipd attach --wsl --busid <busid>
```

此时应该可以在 WSL 中使用 `lsusb` 或类似的指令检查到下位机设备。

```
Bus 001 Device 004: ID a11c:75f3 Alliance RoboMaster Team. RMCS Slave v2.1.2
```

对于接入同一位置的同个下位机，`usbipd bind` 指令只需执行一次。

但每次将设备拔出再插入后，需要再次执行 `usbipd attach` 指令，才能再次在 WSL 中访问。

在 WSL 中完成设备使用后，可物理断开 USB 设备，或运行此命令：

```powershell
usbipd detach --busid <busid>
```


## References

- [Install WSL | Microsoft Learn](https://learn.microsoft.com/en-us/windows/wsl/install)

- [Accessing network applications with WSL | Microsoft Learn](https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking)

- [Connect USB devices | Microsoft Learn](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)

- [What version of Windows am I running? | Microsoft Learn](https://learn.microsoft.com/en-us/windows/client-management/client-tools/windows-version-search)