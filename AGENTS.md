# AGENTS.md

本文件约束所有在本仓库内工作的 agent（codex / claude / opencode / dsh 等）。

## 构建：只用 build-rmcs

```bash
build-rmcs                      # 全量（增量）构建，产物在 rmcs_ws/build 与 rmcs_ws/install
build-rmcs --packages-select rmcs_core   # 只构建某个包（快速迭代用这个）
build-rmcs --packages-select rmcs_core rmcs_bringup
```

**不要**直接运行 `colcon build` / `cmake`：

- 镜像里有两套 CMake —— `build-rmcs` 用 `/opt/cmake/bin/cmake`（4.2.3），
  容器默认 PATH 上是 `/usr/bin/cmake`（3.28.3）。
- CMake 会把自己的版本写进每个 `CMakeFiles/<target>.dir/flags.make`，而 Makefile
  生成器让每个 `.o` 都依赖 `flags.make`。两套 CMake 交替使用会重写 `flags.make`，
  导致该包**全部源码重新编译**（rmcs_core 约 90 个 TU，数分钟）。
- 同理，`build-rmcs` 里已把 `-j$(nproc)` 写进 `MAKEFLAGS`，绕开 colcon 默认追加的
  `-l<nproc>` 负载限速（容器看到的 load average 是宿主机的，机器一忙构建会退化到串行）。

**不要**在仓库根用 `colcon build --base-paths rmcs_ws/src ...` 构建：那会在仓库根
生成第二棵 `build/`、`install/`、`log/` 树，和 `build-rmcs` 的 `rmcs_ws/` 树互不相干。

## 其它约定

- 构建产物固定位置：`rmcs_ws/build`、`rmcs_ws/install`（merge install + symlink install）。
- 测试：在 `rmcs_ws` 下 `colcon test --packages-select <pkg>`，结果用
  `colcon test-result --test-result-base build/<pkg> --verbose` 查看。
