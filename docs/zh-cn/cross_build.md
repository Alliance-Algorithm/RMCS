# RMCS 交叉编译使用说明（第一阶段）

本文档说明 `rmcs-develop:latest-full`、`build-rmcs-cross` 与 CI 的约定。

## 1. 镜像语义

- `qzhhhi/rmcs-develop:latest`：原有开发镜像，行为不变。
- `qzhhhi/rmcs-develop:latest-full`：`latest` 的超集，额外提供 cross 工具链与对向架构 sysroot。

对应关系：

- `latest-full` 的 `linux/amd64` 变体内置 `/opt/sysroots/arm64`
- `latest-full` 的 `linux/arm64` 变体内置 `/opt/sysroots/amd64`

## 2. 仓库内新增能力

- `rmcs_ws/toolchain.cmake`：双向 cross 参数化工具链文件。
- `.script/build-rmcs-cross`：独立 cross 构建入口，不影响现有 `build-rmcs`。

`build-rmcs-cross` 默认输出目录：

- `rmcs_ws/build-cross-<target-arch>`
- `rmcs_ws/install-cross-<target-arch>`
- `rmcs_ws/log-cross-<target-arch>`

## 3. 使用方式

在 `latest-full` 容器里，根据当前容器架构执行对向架构的交叉编译：

```bash
build-rmcs-cross --target-arch arm64
```

适用于 `linux/amd64` 的 `latest-full` 变体。

```bash
build-rmcs-cross --target-arch amd64
```

适用于 `linux/arm64` 的 `latest-full` 变体。

例如，可追加常见 `colcon build` 参数：

```bash
build-rmcs-cross --target-arch arm64 --packages-up-to rmcs_executor
```

## 4. 构建环境隔离约束

`build-rmcs-cross` 会显式清理并重建以下环境，避免 host/target 串用：

- `AMENT_PREFIX_PATH`
- `CMAKE_PREFIX_PATH`
- `COLCON_PREFIX_PATH`
- `PKG_CONFIG_PATH`
- `PKG_CONFIG_LIBDIR`
- `PKG_CONFIG_SYSROOT_DIR`

并且默认启用：

- `--merge-install`
- 不使用 `--symlink-install`

## 5. 本地快速自检

```bash
test -d /opt/sysroots/arm64
command -v aarch64-linux-gnu-gcc
./.script/build-rmcs-cross --target-arch arm64 --packages-up-to rmcs_core
readelf -h rmcs_ws/install-cross-arm64/lib/librmcs_core.so
```

若目标为 `amd64`，将上述 `arm64/aarch64-linux-gnu` 替换为 `amd64/x86_64-linux-gnu`。

本地自检故意覆盖到 `rmcs_core`，用于在推送前尽早暴露 workspace 级联依赖和交叉工具链问题；CI 只保留 `rmcs_executor` 最小 smoke，避免高频业务改动阻塞镜像推送。

## 6. CI 维护约定

- CI 先构建并推送 `qzhhhi/rmcs-base` 双架构 digest。
- 构建 `latest-full` 时显式传入：
  - `SYSROOT_IMAGE_AMD64=<rmcs-base amd64 digest>`
  - `SYSROOT_IMAGE_ARM64=<rmcs-base arm64 digest>`
- `latest-full` 在每个架构 runner 上执行最小 smoke：
  - sysroot 目录存在
  - cross 编译器可执行
  - `build-rmcs-cross --target-arch <opposite> --packages-up-to rmcs_executor` 可完成构建
  - `readelf -h` 校验 `librmcs_executor.so` 架构
