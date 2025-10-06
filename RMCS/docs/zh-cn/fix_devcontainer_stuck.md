# Fix DevContainer Stuck

`Dev Containers` 有时会在启动时卡住长达几分钟，这是一份典型的启动日志：

```
[7515 ms] userEnvProbe is taking longer than 2 seconds. Process not found.
[7978 ms] Stop (3516 ms): Run: docker buildx version
[7980 ms] github.com/docker/buildx 0.12.1 0.12.1-0ubuntu2.1
[7980 ms] 
[7981 ms] Start: Run: docker -v
[8016 ms] Stop (35 ms): Run: docker -v
[8017 ms] Start: Resolving Remote
[8027 ms] Start: Run: git rev-parse --show-cdup
[8035 ms] Stop (8 ms): Run: git rev-parse --show-cdup
[144217 ms] Failed to fetch control manifest: 
[144223 ms] Start: Run: docker ps -q -a --filter label=devcontainer.
...
```

问题的原因是 `Dev Containers` 在每次 reopen 或 rebuild 时会尝试访问 [containers.dev](containers.dev)，但在一些国家或地区，访问请求会被无回复地丢弃，造成 `Dev Containers` 不得不等待到连接超时。

要解决这个问题，可以让网络明确拒绝连接。在主机（不要在 `docker` 容器）的终端中输入：

```bash
echo "127.0.0.1 containers.dev" | sudo tee -a /etc/hosts
```

如果不希望修改 `hosts` 文件，也可以在主机（不要在 `docker` 容器）的终端中输入：

```bash
sudo tee /etc/systemd/system/fix-devcontainer.service <<EOF
[Unit]
Description=Fix vscode devcontainer stuck while loading
After=network.target
StartLimitIntervalSec=0
[Service]
Type=simple
Restart=always
RestartSec=180
User=$USER
ExecStart=bash -c "mkdir -p /tmp/devcontainercli-\$USER && touch /tmp/devcontainercli-\$USER/control-manifest.json"
[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload &&
sudo systemctl enable fix-devcontainer.service &&
sudo systemctl restart fix-devcontainer.service
```

这会添加一个定时任务，每 3 分钟更新 `Dev Containers` 的缓存。因为如果该缓存文件的修改时间[少于 5 分钟，则当前实现会跳过下载清单](https://github.com/devcontainers/cli/blob/f7d4c853bf8c284d784173f3e915a34d961b0b55/src/spec-configuration/controlManifest.ts#L39)。

## References

[Dev container issue: Is it possible to set "fetch control manifest" as optional ? · Issue #8808 · microsoft/vscode-remote-release](https://github.com/microsoft/vscode-remote-release/issues/8808)