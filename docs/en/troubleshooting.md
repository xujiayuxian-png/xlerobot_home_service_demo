# Troubleshooting

Start with the smallest failing boundary.

| Symptom | Check |
|---|---|
| Unsupported or missing config | Copy `config/local.example.yaml`; keep `schema: xlerobot_demo/v1`. |
| Calibration runtime missing | Complete all components, then run `tools/calibrate activate` and `render`. |
| LM Studio unreachable | Load `qwen/qwen3-vl-4b`; test `<lm_studio_url>/v1/models` from both computers and check WSL/LAN firewall rules. |
| Port 8765 fails | Run `tools/doctor gpu`; check the classical Python 3.10 environment, SAM 2 snapshot, and GPD build. |
| Port 8766 fails | Run `tools/doctor gpu`; verify token, CUDA, checkpoint directory, and manifest hashes. |
| ACT returns unauthorized | Put the same `XLEROBOT_ACT_TOKEN` in both local `.env` files. |
| Web console missing | Re-run `tools/setup robot`; it runs `npm ci`, tests, and the production build before colcon. |
| Voice starts but cannot speak | Generate local prompts and verify Edge TTS/network plus the configured ALSA output. |
| No device at `/dev/...` | Fix the stable udev alias; do not replace it with an ambiguous `/dev/ttyUSB*` path. |
| Robot command refused | Real robot and task modes require the literal `--hardware` argument. |

`tools/doctor` checks path existence and HTTP health without opening cameras or
motor devices. If a ROS node fails after startup, keep the first error and the
exact command/config revision; later controller and lifecycle errors are often
consequences rather than the cause.
