# 较为现代化的机甲大师自瞄

## 部署步骤

先确保海康相机的 SDK 正确构建，再保证 `rmcs_exetutor` 正确构建，如果要运行 RMCS 控制系统的话

```sh
# 进入工作空间的 src/ 目录下
git clone https://github.com/Alliance-Algorithm/ros2-hikcamera.git --branch 2.0 --depth 1
git clone https://github.com/Alliance-Algorithm/rmcs_auto_aim_v2.git

# 构建依赖
build-rmcs
```

## 项目架构

### 文件排布

### 任务调度

## 调试指南

### OPENCV 可视化窗口

如果你使用英伟达 GPU，那你的 OPENCV 的可视化可能会在这一步被拿下，比如`cv::imshow`，设置环境变量永远使用 CPU 渲染即可

```
# F*** Nvidia
export LIBGL_ALWAYS_SOFTWARE=1
```

### 视频流播放

诚然，依靠 ROS2 的 Topic 来发布 `cv::Mat`，然后使用 `rviz` 或者 `foxglove` 来查看图像不失为一个方便的方法，但经验告诉我们，网络带宽和 ROS2 的性能无法支撑起高帧率高画质的视频显示，所以我们采用 RTP 推流的方式来串流自瞄画面，这会带来一些的延迟（大概1s吧），但推流完全支撑得起 100hz 以上的流畅显示，且在较差的网络环境也能相对流畅地串流，这是 ROS2 不能带给我们的良好体验，至于延迟，我想也没有人同时看着枪口和视频画面调参吧，网络较好的情况下，延迟不足 1s

首先打开`config/config.yaml`文件，将 `use_visualization`设置为`true`，然后配置推流参数：

```yaml
visualization:
    # ......
    framerate: 80
    monitor_host: "127.0.0.1"
    monitor_port: "5000"
    # ......
```

需要配置的只有帧率和主机网络地址，帧率需要同`capturer`模块的帧率一致（也许应该自动读取相机的帧率，以后再论吧），`host`填自己电脑的 IPv4 地址（注意是和运行自瞄的主机同一局域网下的地址），端口随意，别和本机服务冲突就行了

然后启动项目：

```sh
ros2 launch rmcs_auto_aim_v2 launch.py
```

如果相机正常连接的话，且推流模块正确加载，则会看到以下日志：

```
[...] [INFO] [...] [Capturer]: Connect to capturer successfully
[...] [INFO] [...] [visualization]: Visualization session is opened
[...] [INFO] [...] [visualization]: Sdp has been written to: /tmp/auto_aim.sdp
```

接下来只需要将 `/tmp/auto_aim.sdp` 文件拷贝到自己电脑上，使用能够打开`SDP`文件的视频播放器打开即可，也可以使用指令：

```sh
# 如果自瞄运行在机器人上，就加上 --remote 参数
play-autoaim --user username [--remote][--no-copy]
```

随后你会看到这样的输出：

```
/workspaces/RMCS/main/RMCS (main*) » play-autoaim --user creeper                                       ubuntu@creeper
creeper@localhost's password: 
auto_aim.sdp                                                                       100%   70   330.8KB/s   00:00    
✅ 文件已拷贝到宿主机：/tmp/auto_aim.sdp
creeper@localhost's password: 
---------------------------------------------------------------------------------------------------------------------
/workspaces/RMCS/main/RMCS (main*) » [0000565203226630] main libvlc: 正在以默认界面运行 vlc。使用“cvlc”可以无界面-
```

电脑便自动打开 VLC 播放视频流了，SDP 文件会经过`机器人 -> 容器 -> 本地`到 `/tmp/auto_aim.sdp/` 目录

脚本默认使用 VLC 作为视频流播放器，默认延迟较高，可以将播放器的播放缓存设置为 0 来获得**低延迟的串流体验**：`Tools -> Preferences -> search 'caching'`

愉快调试吧！
