# XLeRobot 家庭服务 Demo

[English](README.md)

这个仓库记录的是一台经过改造的双轮 XLeRobot：听到取物请求后导航到桌边，抓取
羽毛球，寻找最近的人并完成递送。它不是通用机器人框架，也不是大而全的课程。

**先看完整真机 Demo：**
[XLeRobot 取物递送视频（Bilibili）](https://www.bilibili.com/video/BV1srNg6XEZj)。

最值得直接参考的是整机标定流程和两条抓取路线：

```text
物体请求
├── 传统路线：VLM/SAM2 + RGB-D 几何 ── centroid 或可选 GPD 目标
└── 混合 ACT：标定后的 MoveIt 预抓取 ── 腕部图像 ACT 动作块
                                      │
                          机器人本地校验与执行
                                      │
                              抓取复核与递送
```

默认 Demo 是 `act` + `羽毛球`，语音和 Web 控制台默认开启。ACT 模型只用 30 条
黄色胶棒示教训练，因此羽毛球运行只是定性的分布外泛化展示，不代表通用抓取能力；
`centroid` 和 `gpd` 用于对比传统路线。

## 1. 唯一支持的参考硬件

- Robot 端：Ubuntu 24.04 x86_64、ROS 2 Jazzy。
- 双轮差速 XLeRobot 改装：两个 Feetech 轮舵机、双 SO-101 类机械臂、云台头、
  2D 激光雷达、头部 RealSense D455 和右腕 USB 相机。
- GPU 端：Ubuntu 24.04 WSL2，已验证 RTX 3080。
- LM Studio 提供 `qwen/qwen3-vl-4b`；本仓运行传统抓取服务 `8765` 和 ACT
  服务 `8766`。

普通 XLeRobot 的接线、舵机 ID、坐标系和相机支架不一定相同，请先看
[硬件改造说明](docs/zh-CN/hardware.md)。

## 2. Robot/GPU 双机安装

GPU 和 Robot 两台电脑都克隆同一份源码：

```bash
git clone --recurse-submodules <repository-url>
cd xlerobot_home_service_demo
cp config/local.example.yaml config/local.yaml
cp .env.example .env
```

编辑两个本地文件。两台电脑使用同一个 ACT token，`config/local.yaml` 中填写
Robot 能访问的 GPU 地址。

GPU 端先启动 LM Studio，再安装两个固定版本的 Python 环境：

```bash
./tools/setup gpu
# 如果要使用可选的 GPD 原生候选生成器，请加 --with-gpd。
```

首个 Hub revision 发布前，须把已验收的本地 checkpoint 连同生成的
`model-manifest.json` qualification 记录复制到 `models.act_checkpoint` 与
`models.act_manifest` 配置的路径。immutable Hub revision 发布后，这一步可由
`tools/act download` 完成。缺少任一可校验来源时 doctor 会有意失败。

Robot 端安装 ROS 依赖、Web 控制台、语音运行时、可选 AGPL 人体检测器和本地
语音提示：

```bash
./tools/setup robot --with-person-detector --with-kws-model --generate-voice-prompts
```

默认语音需要显式 `--with-kws-model`。该参数会先提示 KWS 权重/词表的上游条款
仍不明确，再直接从原提供者下载；本仓既不打包也不镜像这些文件。完整步骤见
[安装说明](docs/zh-CN/install.md)。

## 3. 运行 doctor

标定或启动 Demo 前，在两台电脑分别执行只读检查：

```bash
./tools/doctor gpu
./tools/doctor robot
```

它检查精确环境、模型摘要、服务状态、本地资产和稳定设备名，但不会打开相机或电机
设备。每个 `ERROR` 都指向失败边界；常见修复见[排障说明](docs/zh-CN/troubleshooting.md)。

## 4. 标定当前机器

不要照抄另一台机器的数值。公开流程按下面顺序生成不可变 bundle：

```text
servo -> head-camera -> right-handeye
base -------------------------------> grasp-alignment -> activate -> render
```

先执行 `./tools/calibrate status`。真机采集统一使用
`./tools/calibrate capture <workflow> --hardware`；两组相机 replay 不需要硬件。五个
组件全部通过后执行：

```bash
./tools/calibrate activate
./tools/calibrate render
./tools/calibrate status
```

Demo 只读取摘要匹配的 active runtime。标靶、采样、质量门、续采和回滚详见
[标定说明](docs/zh-CN/calibration.md)。

## 5. 分别运行 centroid、GPD 和 ACT

先在 GPU 启动提议服务，在 Robot 启动一次已标定的运行栈：

```bash
# GPU 端
./tools/run gpu

# Robot 端，终端 1
./tools/run demo --hardware
```

再从 Robot 的第二个终端为每次请求显式选择一个 backend：

```bash
./tools/run grasp centroid --hardware
./tools/run grasp gpd --hardware
./tools/run grasp act --hardware
```

三条命令运行同一取物递送任务，但锁定抓取 backend。GPD 失败会明确报错，不会静默
变成 centroid；传统路线只承诺当前 top-grasp 范围。详见
[两条抓取路线](docs/zh-CN/grasping.md)。

## 6. 运行完整语音与 Web Demo

`./tools/run demo --hardware` 启动八阶段 Robot 栈，本身不自动提交任务：

```text
定位 -> 导航/贴桌 -> 目标感知 -> 抓取与验证
-> 寻人 -> 接近 -> 语音反馈 -> 递送
```

对着麦克风说出请求，或打开 `http://<robot-host>:8080`。两种入口默认使用 `act` 和
`羽毛球`；Web 也能显式选择 `centroid`、`gpd`。任务记录同时保存请求值和实际
backend。详见[完整 Demo](docs/zh-CN/demo.md)。

## 7. 排障、资产与仓库边界

仓库包含 ROS 源码、配置模板、标定求解与回放样例、两套抓取实现、ACT HTTP
服务，以及五个顶层入口：`setup`、`doctor`、`calibrate`、`act`、`run`。
不提供仿真或 Mock 框架。

Git 中不包含家庭地图、单机标定、录音、模型权重、凭据和历史预录 MP3。模型 ID、
已知摘要和可用状态记录在[资产清单](assets/models/manifest.yaml)中。ACT 模型确定为
Apache-2.0，30 条示教数据确定为 CC BY 4.0；两者的 Hub repo ID 已固定，但首次上传
仍待完成。

ACT 数据到模型的流程只使用一个入口：

```bash
./tools/act collect --hardware
./tools/act convert --dry-run
./tools/act train --dry-run
./tools/act evaluate --checkpoint PATH --output PATH/model-manifest.json
```

第五个操作 `tools/act download` 只在公开 manifest 写入首次 immutable Hub
revision 后才可用。完整说明见 [ACT 数据到模型流程](docs/zh-CN/act-workflow.md)。

两阶段 ACT 行为可直接看
[ACT 路线视频](https://www.bilibili.com/video/BV18RK66JEdP)。

项目自有代码和文档使用 Apache-2.0。第三方代码、模型和生成资产使用各自条款，
详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) 和
[模型说明](assets/models/external-models.md)。
