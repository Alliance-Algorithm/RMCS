# Build Docker Image

于 Repo 根目录下，在终端中分别使用：

```bash
docker build . --target rmcs-runtime -t rmcs-runtime:latest
```

```bash
docker build . --target rmcs-develop -t rmcs-develop:latest
```

构建开发容器和部署容器。

需要注意的是，部分国家和地区会阻断对 `github` 的连接，此时构建需要使用代理：

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

如果不使用本机回环地址 (127.0.0.1) 作为代理地址，构建参数中 `--network host` 可以省去。