# XLeRobot Home Service Demo

[English](README.md) · [文档目录](docs/zh-CN/README.md)

**对 XLeRobot 稍加改装，实现听指令、到桌边抓取物品、再递送给人的 Demo，
并公开配套代码和工具，方便参考复现。**

[完整真机视频](https://www.bilibili.com/video/BV1srNg6XEZj)
· [ACT 抓取](https://www.bilibili.com/video/BV18RK66JEdP)
· [网页操作](https://www.bilibili.com/video/BV1GSK66XEqf)

重点是当前两轮参考机上的**标定和两条抓取路线**，不是通用框架或教学课程：

- **混合 ACT，主 Demo 路线：** RGB-D 目标 → 标定后的 MoveIt 预抓取 →
  腕部图像 ACT action chunks → 机器人本地执行。
- **传统几何路线：** VLM + SAM 2 + RGB-D → centroid 或 GPD 顶抓计划 → MoveIt 执行。
  GPD 在 GPU 主机运行，失败不会静默切换到 centroid。

两条路线共用机器人运行栈、标定和任务流程。默认 `act` + `羽毛球`，语音和网页开启。
ACT 权重**只使用 30 条黄色胶棒示教训练**；羽毛球抓取属于定性泛化演示，
不声明成功率或通用抓取能力。

## 1. 对齐参考硬件

- 机器人：两轮改装 XLeRobot、双 SO-101 风格机械臂、云台头、D455、右腕 USB 相机、
  LD06 风格雷达、麦克风和扬声器。
- Robot 主机：Ubuntu 24.04 x86_64 + ROS 2 Jazzy。
- GPU 主机：NVIDIA Ubuntu/WSL2；参考环境为 Ubuntu 24.04 WSL2 + RTX 3080。
- VLM：LM Studio 提供 `qwen/qwen3-vl-4b`。
- 单个右臂 Leader **仅在自行采集数据时需要**。

先看 [BOM、接线与安装说明](docs/zh-CN/hardware.md)。本项目的配置不适用于所有原版 XLeRobot。

## 2. Robot/GPU 双机安装

两台电脑克隆同一仓库，复制并修改本机配置：

```bash
git clone --recurse-submodules REPOSITORY_URL xlerobot_home_service_demo
cd xlerobot_home_service_demo
cp config/local.example.yaml config/local.yaml
cp .env.example .env
```

设备路径、局域网地址和文件位置填在 `config/local.yaml`；两台 `.env` 使用相同 ACT token。
前置依赖和填写清单见[安装说明](docs/zh-CN/install.md)。

```bash
# GPU 主机
./tools/setup gpu
# 如需原生 GPD 后端，改用 ./tools/setup gpu --with-gpd

# Robot 主机
./tools/setup robot --with-person-detector --with-kws-model --generate-voice-prompts
```

完整语音和寻人 Demo 需要这些显式附加项：person detector 采用 AGPL-3.0；KWS 模型条款
尚未明确，由工具直接从提供方下载，不随本仓库分发。详情见安装说明和第三方声明。

**资产状态：** ACT 权重和数据已私有上传，尚未公开，下载情况见
[模型与数据](docs/zh-CN/assets.md)。发布前需要已经验证的本地 checkpoint 及 manifest。
使用发布权重运行 Demo **不需要先自行数采或训练**。

## 3. 检查依赖

```bash
./tools/doctor gpu
./tools/doctor robot
```

Doctor 只读检查，不打开相机或电机。首次安装时，缺标定、缺地图和服务尚未启动
表示后续步骤还没完成；按下面的流程补齐，启动 Demo 前再次检查。

## 4. 标定，然后准备地图与地点

按[标定说明](docs/zh-CN/calibration.md)完成：

```text
servo → head-camera → right-handeye ─┐
base（独立实测）────────────────────┴→ grasp-alignment → activate → render
```

随后[建图、保存 table 地点、验证并激活场地](docs/zh-CN/mapping.md)。
标定和场地准备是两件事：标定完成并不代表机器人已经知道地图和桌子的位置。
不要复制别人的设备标定或家庭地图。

## 5. 分别运行三个抓取后端

```bash
# GPU：先启动 LM Studio，再执行
./tools/run gpu

# Robot 终端 1
./tools/run demo --hardware

# Robot 终端 2：每轮只执行其中一条
./tools/run grasp act --hardware
./tools/run grasp centroid --hardware
./tools/run grasp gpd --hardware
```

`grasp` 会固定后端并提交**完整取物递送任务**，不是仅移动机械臂的独立抓取命令。
目标物品读取 `demo.object_id`。见[两条抓取路线](docs/zh-CN/grasping.md)。

## 6. 完整语音 + 网页 Demo

`tools/run demo --hardware` 启动后等待请求，不会自动发起任务。
打开 `http://<robot-host>:8080`，或说“小乐小乐”后发出取物指令。
详见[完整 Demo 操作说明](docs/zh-CN/demo.md)。

```text
定位 → 导航/贴桌 → 目标感知 → 抓取与验证
→ 寻人 → 接近 → 语音反馈 → 递送
```

## 7. 配套工具、排障与资产

想采集自己的示教，请走独立的 [ACT 数采 → 转换 → 训练流程](docs/zh-CN/act-workflow.md)。
正常操作为 **Start → Home → End → 下一条**。有效数据默认保留；End 后仍可遥操放下物品。

| 入口 | 用途 |
| --- | --- |
| `tools/setup robot\|gpu` | 源码安装 |
| `tools/doctor robot\|gpu` | 检查本机依赖与服务 |
| `tools/calibrate` | 标定采集、求解、激活、回放和回滚 |
| `tools/act` | 数采、转换、训练、检查和下载 |
| `tools/run` | GPU 服务、建图、指定后端任务和完整 Demo |

[常见问题](docs/zh-CN/troubleshooting.md) · [模型与数据](docs/zh-CN/assets.md)
· [源码布局](docs/zh-CN/README.md#源码布局)

本机配置、地图、标定、示教、权重和日志不提交到 Git，运行资产通常放在被忽略的
`.xlerobot/`。不提供仿真/Mock 框架、交付安装包或 systemd 安装。

项目代码与文档：[Apache-2.0](LICENSE)。ACT 权重：Apache-2.0。30 条示教数据：CC BY 4.0。
第三方资产保留各自条款，见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
