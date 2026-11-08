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

## 组件接线：必须检查循环依赖

- 修改组件输入/输出、partner component、反馈链或 YAML 接线后，必须检查实际加载组件的依赖图；编译通过、配置解析通过和单个模型推理通过，不代表执行器能够启动。
- 按接口的真实生产组件建立“生产者 → 消费者”有向边，包含 partner component、`before_pairing()` 中动态注册的输入和 YAML 配置产生的输入。可选输入在生产者存在时同样建立依赖；不能靠 `required=false` 消除环。
- 对完整加载集合执行拓扑排序或等价的有向环检查，尤其检查 reset、enable、状态反馈、上一动作、观测和动作发布链。不要只检查 YAML 列表顺序；执行器按依赖排序，不按列表顺序更新。
- 禁止形成“命令组件读取状态估计 → 状态估计读取该命令组件的 reset”之类的环。必要时将 reset 等前置信号移到只依赖硬件/遥控输入的独立组件或 partner component；保留明确接线和更新顺序。
- 修复依赖环时不得通过删除安全反馈、绕过 reset、伪造默认有效状态，或用直接绑定偷偷绕过执行器排序来掩盖问题。复位必须仍能及时停止输出、清空内部状态并作废旧推理结果。
- 优先增加不连接硬件、不使能执行器的配对与依赖图回归测试，覆盖真实 YAML 和 partner components；测试替身必须保留相关生产组件的接口归属与依赖。仅静态搜索或单组件测试不能声称完整组件图已经通过验证。
- 启动报 `Circular dependency found` 时，收集从该行开始、包含全部 `Component [...]` 标题及其 `Depends on [...]` 的日志。下游日志/可视化组件也可能被环连带阻塞，不能只凭最后一条依赖认定根因。
- 交付时分别说明构建、配置解析、依赖图/配对、reset 回归和实机验证的结果；未执行的验证必须明确标注。没有明确授权，不为验证接线启动真实硬件控制。

## 注释：一行，必要时才写

- 代码注释**只允许一行**，且只写在非显而易见处（协议字节偏移、魔数、与手册/硬件的对应关系）。
- 禁止多行说明块、分段解释、分点罗列，也禁止把改动理由、历史沿革写进代码：理由进提交信息或
  `.codex/report/`。`///` 文档注释同样一行。
- 不要顺手重排未改动代码里的注释。

## 其它约定

- 重构、部署、训练、验证和审查报告写入本机 `.codex/report/`，不提交到仓库，不放在 `docs/`。
- `docs/` 维护正式项目文档；不要为本机报告添加 `.gitignore` 放行规则或 README 链接。
- 构建产物固定位置：`rmcs_ws/build`、`rmcs_ws/install`（merge install + symlink install）。
- 测试：在 `rmcs_ws` 下 `colcon test --packages-select <pkg>`，结果用
  `colcon test-result --test-result-base build/<pkg> --verbose` 查看。
