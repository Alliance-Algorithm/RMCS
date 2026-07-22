# Build Docker Image

于 Repo 根目录下，在终端中分别使用：

```bash
docker build . --target rmcs-base -t rmcs-base:latest
```

```bash
docker build . --target rmcs-develop -t rmcs-develop:latest
```

```bash
docker build . --target rmcs-runtime -t rmcs-runtime:latest
```

构建基础容器、开发容器和部署容器。

## 构建 `rmcs-develop:latest-full`

`latest-full` 需要对向架构的 `rmcs-base` 作为 sysroot 来源。

下面示例以在 `amd64` 主机上构建 `amd64 latest-full` 为例：

```bash
docker buildx build . --platform linux/amd64 --target rmcs-base -t rmcs-base:linux-amd64 --load
```

```bash
docker buildx build . --platform linux/arm64 --target rmcs-base -t rmcs-base:linux-arm64 --load
```

```bash
docker buildx build . --platform linux/amd64 --target rmcs-develop-full -t rmcs-develop:latest-full \
  --build-arg SYSROOT_IMAGE_AMD64=rmcs-base:linux-amd64 \
  --build-arg SYSROOT_IMAGE_ARM64=rmcs-base:linux-arm64 \
  --load
```

如果是在 `arm64` 主机上构建，则交换目标平台即可（`latest-full` 的 sysroot 方向相反）。

## 代理构建

部分国家和地区会阻断对 `github` 的连接，此时构建需要使用代理：

```bash
docker build . --target rmcs-runtime -t rmcs-runtime:latest \
  --network host \
  --build-arg HTTP_PROXY=http://127.0.0.1:7890 \
  --build-arg HTTPS_PROXY=http://127.0.0.1:7890
```

```bash
docker build . --target rmcs-develop -t rmcs-develop:latest \
  --network host \
  --build-arg HTTP_PROXY=http://127.0.0.1:7890 \
  --build-arg HTTPS_PROXY=http://127.0.0.1:7890
```

请自行把 `http://127.0.0.1:7890` 改为合适的代理地址。

如果不使用本机回环地址 (`127.0.0.1`) 作为代理地址，构建参数中的 `--network host` 可以省去。
