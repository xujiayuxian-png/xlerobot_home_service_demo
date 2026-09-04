# 排障

| 现象 | 检查 |
|---|---|
| 配置缺失/不支持 | 复制 `config/local.example.yaml`，保留 `schema: xlerobot_demo/v1`。 |
| calibration runtime 缺失 | 补齐分项后执行 `tools/calibrate activate` 和 `render`。 |
| LM Studio 不通 | 加载 `qwen/qwen3-vl-4b`；两台电脑分别测试 `/v1/models` 并检查 WSL/LAN 防火墙。 |
| 8765 启动失败 | `tools/doctor gpu`，检查 Python 3.10、SAM 2 snapshot 和 GPD 构建。 |
| 8766 启动失败 | `tools/doctor gpu`，检查 token、CUDA、checkpoint 和 manifest 摘要。 |
| ACT unauthorized | 两台电脑 `.env` 中使用相同 `XLEROBOT_ACT_TOKEN`。 |
| Web 页面缺失 | 重跑 `tools/setup robot`；它会先 `npm ci`、测试和 build，再 colcon。 |
| 语音无输出 | 本地生成 prompts，检查 Edge TTS/网络和 ALSA 输出设备。 |
| `/dev/...` 不存在 | 修复稳定 udev 别名，不要改成含糊的 `/dev/ttyUSB*`。 |
| Robot 命令被拒绝 | Robot 与 task 模式必须包含字面参数 `--hardware`。 |

`tools/doctor` 只检查路径和 HTTP health，不打开相机或电机设备。ROS 启动失败时优先
保留第一条错误及对应命令/配置版本，后续 controller/lifecycle 错误往往只是连锁结果。
