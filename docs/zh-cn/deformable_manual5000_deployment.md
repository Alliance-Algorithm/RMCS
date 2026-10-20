# Deformable manual_5000 模型部署

`deformable-infantry-omni-rl.yaml` 使用训练目录
`deformable_guarded_mlp5_v4/2026-10-16_17-39-12_manual_5000/model_4999.pt`
对应的 `models/deformable_guarded_mlp5_manual_5000.onnx`。

- 通过训练项目的 `scripts/rsl_rl/export_onnx.py` 导出，不加载 Isaac 环境。
- 传感器路由 MLP，输入 `obs: float32[1,160]`，输出 `actions: float32[1,4]`。
- 200 Hz 推理，五帧最旧到最新的 32 维观测，历史间隔 10 ms；动作反馈保留相同延迟。
- 无观测归一化；观测裁剪 ±10，动作裁剪 ±1。RF/LF/LB/RB 顺序，电流反馈为原始计数除以 2048。
- 动作使用 `minangle_physical_v3` 非对称映射，平地基准为 17°。模型外的原生悬挂保护执行训练中的平地确认、协调回位和末端制动。
- ONNX 布局哈希为 `0x699f7a77fc3d9462`，文件模型 ID 为 `0x7542eb585bbdd224`；桥通过 `expected_model_id` 拒绝其它模型。

训练权重 SHA256：`9fbce1a0549bfb6957db252b7357834027d524d55dc519801d38028a20a7772d`。
完整来源和数值校验记录在 `rmcs_rl/models/deployment/deformable_guarded_mlp5_manual_5000.json`。
ONNX 只包含 actor，原生保护和历史处理是部署契约的一部分。

在训练项目根目录复现导出：

```bash
python scripts/rsl_rl/export_onnx.py \
  --checkpoint logs/rsl_rl/deformable_guarded_mlp5_v4/2026-10-16_17-39-12_manual_5000/model_4999.pt \
  --output_dir exported/manual_5000 --filename deformable_guarded_mlp5_manual_5000.onnx --no_jit
```

导出后须使用 RMCS 的 `tool/stamp_layout_metadata.py` 对照这份 YAML 盖章；
若重新导出或修改元数据，应重新计算模型 ID 并同步 `expected_model_id`。
验证部署文件：

```bash
python rmcs_ws/src/rmcs_rl/tool/check_policy_contract.py \
  rmcs_ws/src/rmcs_rl/models/deformable_guarded_mlp5_manual_5000.onnx \
  --config rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni-rl.yaml \
  --expect-model-id 0x7542eb585bbdd224
```

双下复位、禁用及策略失效会清空历史、动作反馈和回位轨迹；无效传感器、模型不匹配或过期动作均不取得悬挂控制权。
离线数值测试与容器构建不代表实车验证，实车启动由操作者执行。
