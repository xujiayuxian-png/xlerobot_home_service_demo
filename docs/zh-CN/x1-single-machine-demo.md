# X1 单机 Demo：启动与现场测试

2026-09-15，`x1-native`。本机 `config/local.yaml` 已选择全端侧推理配置。
运行不需要启动笔记本或 GPU 服务；训练、历史数据传输配置不属于运行依赖。

## 晚上怎么启动

在 X1 的项目目录执行：

```bash
tools/run demo --hardware
```

命令启动手动 systemd 服务，依次加载本地 VLM、人员检测、ASR、ACT，检查就绪后
启动原 ROS demo。无需另开四个模型终端。模型加载期间网页可能尚未启动，
可在另一终端观察：

```bash
journalctl -u xlerobot-demo -f
```

模型就绪后的 ROS 启动详情在 `.xlerobot/npu/stack/demo.log`，可用
`tail -f .xlerobot/npu/stack/demo.log` 查看；各模型日志也在该目录。

网页地址为 `http://<X1地址>:<config/local.yaml 中的 demo.web_port>`。
页面就绪后使用原来的唤醒词和取物指令，也可在网页发起任务。任务过程中通过网页
取消，固定提示音、定位检查、碰撞监测与控制频率沿用现有逻辑。

停止和查看状态：

```bash
tools/run demo --stop
tools/run demo --status
```

停止会清理模型和 ROS 子进程；模型进程意外退出也会结束整组运行。
服务不开机自动启动，不自动重启任务。

## 实际运行的组件

| 功能 | X1 本地实现 |
| --- | --- |
| 唤醒、播报 | 原轻量唤醒 + 六条固定 PCM 音频 |
| 中文转写 | Whisper-base W8A16 / NPU |
| 意图、物品框选、抓取视觉判断 | 同一个 Qwen2.5-VL-3B W4A16 / NPU |
| 人员检测 | YOLOv8n W8A8 / NPU |
| 抓取动作推理 | 原 ACT 权重，腕部 640×480，HTP FP16，完整 100×6 动作块 |
| 导航、定位、控制与必要前后处理 | 原 X1 CPU 路径 |

本地配置只允许 ACT 抓取路径，不启动或探活 SAM2/GPD。所有推理地址强制使用
回环地址，配置混入远程模型地址时启动检查会报错，不静默回退 GPU。
模型进程仅返回推理数据，电机命令仍由 ROS 控制节点负责。

3B 输入固定为 672×672，框坐标映射回相机原图；NPU 请求使用 0.1 温度，避免
本机 SDK 在零温度下退出。抓取判断使用简短提示，保留原机械判据和视觉阈值。

## 本次短验证结果

- 四个相关 ROS 包构建成功；语音、感知、任务的软件测试通过。
- 新增/相关 NPU 客户端测试 15 项、demo 启动参数与保护测试 14 项通过。
- `tools/doctor robot` 在本地模型就绪时为 0 错误；未连接 Leader 的提示不阻止 demo。
- 保存音频的本地转写与意图解析得到羽毛球取物任务，取消指令被拒绝为新取物任务。
- 保存图像的圆筒框选约 3.58 秒，坐标已映射回原图；该空夹样本被判为未抓住，
  约 2.71 秒。这个结果不替代其他场景下的抓取判断质量。
- 人员检测得到三个人框；原 ACT 客户端收到完整、有限的 100×6 动作，约 54 ms。
- 实际停止一个模型进程，确认监督进程会清理其他模型进程。

本轮没有打开电机、启动导航或执行真实抓取递送。默认配置和启动链路已完成端侧
接入，**完整实机演示是否成功，需要今晚现场走一遍确认**。不以模型接口通过
或 CPU 数值替代实机结果；3B 抓取视觉判断的已知误报仍记录在实验文档中。

## 每次测试自动记录资源

全端侧 `tools/run demo --hardware` 自动启动只读记录器，从模型加载前开始，
随 demo 停止，不需要另开监测命令。每次启动独立保存到
`.xlerobot/performance/<UTC时间>-<进程号>/`，`latest` 指向最近一次 demo 记录。
记录器退出时监督进程会结束整组运行，避免以为在记录但实际没有数据。

| 文件 | 内容 |
| --- | --- |
| `samples.jsonl` | 约每秒一条：整机 CPU、demo 进程树 CPU/RSS、各进程 CPU/RSS/线程数、其他高占用进程、可用内存、Swap、温度、CPU 频率、控制线程调度状态，以及任务 ID/阶段和语音状态 |
| `events.jsonl` | 任务阶段、结果/错误码、语音状态、事件流连接/断连的时间戳，保留不足一秒的阶段切换；不记录语音原文和图像 |
| `summary.json` | 每秒更新：整场及各能力阶段/语音状态的平均 CPU、单样本峰值、内存峰值，整场最高 10 秒窗口 CPU |
| `metadata.json` | 采样口径、开始时间和记录器 PID |
| `dsp-mon.log` | 本机 `dsp_mon` 的原始 QDSP6、HVX、HMX 利用率和 DSP 频率输出 |

查看当前汇总：

```bash
cat .xlerobot/performance/latest/summary.json
```

整机及 demo CPU 均以**所有在线 CPU 合计为 100%**；单进程字段
`cpu_one_core_percent` 则以一核为 100%，不要直接混用。模型进程标注为
`vlm`、`whisper`、`person`、`act`，ROS 子进程标为 `demo` 并保留进程名。
RSS 相加会重复计算共享页，应结合整机可用内存判断。温度、频率等较慢指标
每五秒刷新一次。

已找到并验证本机 `/usr/bin/dsp_mon`，记录器现在自动以一秒窗口读取 QDSP6、
HVX、HMX 利用率和 DSP 频率，写入每条样本的 `dsp` 字段；分阶段汇总也包含
三个利用率的平均值和峰值。三个指标保留工具原始口径，不能相加成一个“NPU 总利用率”，
也不能仅凭 HMX 未满就认定推理还有同比例加速空间。缺失、退出或超过三秒未更新时
标为不可用，不填零。监测通过 FastRPC 读取 DSP 状态，不访问电机。

2026-09-15 短验证：空闲时 HVX/HMX 为 0%；连续执行 ACT 六秒、421 次推理时，
QDSP6 峰值约 97.9%、HVX 约 45.8%、HMX 约 35.8%，随后回到空闲。
这是独立推理压力测试，**不是完整 demo 的负载**。原始证据在忽略目录
`.xlerobot/performance/npu-counter-check/`。之前“没有 NPU 计数器”的结论仅基于
常见系统接口检查，已被这次专用工具实测修正。

手动查看可执行 `dsp_mon --all --follow`；正常 demo 已自动采集，无需重复启动。
软件测试使用记录器的 `--no-dsp-monitor`，避免接触 DSP 设备。

资源采样按区间结束时的阶段标注；跨阶段的那一秒可能混合两个阶段的开销。
不足一秒的阶段有事件记录，但可能没有独立资源样本。事件流断开时标为
`startup_or_telemetry_unavailable`，不能算作正常待机。可以通过任务 ID 和事件时间
回看单轮导航、识别、抓取、验证及递送，不能仅凭整场平均数判断。

记录器不订阅相机，不发任务或运动指令，不保存凭据。原始记录逐行落盘，
汇总原子替换；正常停止会写入 `finished: true`，强杀时保留已有数据。
记录在忽略目录内，保留多次测试供比较，不自动删除旧测试记录。

## 独立检查与回退

如需在不启动机器人时检查模型，可先运行 `tools/run npu`，另开终端执行
`tools/doctor robot`；完成后用 `tools/run npu --stop` 清理，再启动正式 demo，
避免重复占用模型端口。该诊断入口不允许 `--hardware`。

本机已安装 `tools/setup robot --install-npu-device-rule` 对应的 udev 别名规则。
它保留原 FastRPC 设备权限；本轮已验证别名创建及随后模型加载，未进行整机重启。
SDK 与模型均已在本机准备，运行时不下载模型或生成语音。

切换前配置保存在忽略目录 `.xlerobot/npu/migration/local-before-single-machine.yaml`。
需要退回之前的 CPU/远程推理配置时，先停止 demo，再恢复该文件：

```bash
tools/run demo --stop
cp .xlerobot/npu/migration/local-before-single-machine.yaml config/local.yaml
```

地图、标定和设备配置没有重新生成。具体短测记录保存在忽略目录
`.xlerobot/npu/migration/`。较早的探索过程见[端侧推理实验记录](x1-npu-integration.md)。
