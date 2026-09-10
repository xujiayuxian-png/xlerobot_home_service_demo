<h1 align="center">小乐机器人 · XLeRobot 版</h1>

<p align="center"><sub>XLeRobot Home Service Demo</sub></p>

<p align="center">
  对 XLeRobot 稍加改装，听一句指令，把物品送到你手边。<br/>
  <strong>公开 Demo、标定工具和抓取后端，方便你参考复现。</strong>
</p>

<p align="center">
  <a href="README.md">English</a> ·
  <a href="docs/zh-CN/README.md">文档目录</a> ·
  <a href="https://huggingface.co/lissajous/xlerobot-act-local-grasp-v1">ACT 权重</a> ·
  <a href="https://huggingface.co/datasets/lissajous/xlerobot-glue-stick-grasp-30">30 条示教数据</a>
</p>

<p align="center">
  <a href="https://www.bilibili.com/video/BV1srNg6XEZj"><strong>▶ 完整真机视频</strong></a> &nbsp;·&nbsp;
  <a href="https://www.bilibili.com/video/BV18RK66JEdP">ACT 抓取</a> &nbsp;·&nbsp;
  <a href="https://www.bilibili.com/video/BV1GSK66XEqf">网页操作</a>
</p>

<p align="center">
  <img src="docs/images/robot-hero.png" width="100%" alt="基于仓库 XLeRobot URDF 的棚拍风格可视化"/>
  <br/><sub>URDF 可视化，并非实机照片。<a href="docs/artwork/README.md">来源与渲染方法</a> · 实际运行请看上方视频。</sub>
</p>

## 一台机器人，两条抓取路线

同一套标定和机器人运行栈，完成到桌边、抓取、寻人、递送的完整任务。
切换的是抓取后端，不是另一套 Demo。

本项目只完成了**右臂实机 Demo**：手眼标定、悬停验证和 ACT 数采均针对右臂，
头部相机标定为这条流程服务。左臂仅顺带完成舵机标定，不代表左臂或双臂 Demo 已验证。

![共用机器人运行栈的混合 ACT 与传统几何抓取路线](docs/images/grasp-routes.svg)

- **混合 ACT —— 主 Demo 路线。** RGB-D 目标 → MoveIt 预抓取 →
  腕部图像 ACT action chunks → 机器人本地执行。
- **传统几何 —— 两个可替换后端。** VLM + SAM 2 + RGB-D →
  `centroid` 或 `gpd` 顶抓计划 → MoveIt 执行。GPD 在 GPU 主机运行，
  失败不会静默切换到 centroid；支持范围是顶抓，不是通用 6-DoF 抓取。

默认 **ACT + 羽毛球**，语音和网页开启。发布权重**只使用 30 条黄色胶棒示教训练**。
羽毛球抓取属于定性泛化演示，不代表统计成功率或通用抓取能力。

## 复现这个 Demo

主 Demo 和工具页面顶部都提供 **EN / 中文** 切换按钮。默认保持中文，浏览器会记住上次选择；
物体名、任务 ID、路径、测量数值和数采指令不会被翻译。切换不重启设备会话，也不改变语音链路的语言；
未识别的服务诊断和原始日志保留原文。

### 1 · 对齐参考硬件

| 部分 | 参考配置 |
| --- | --- |
| 机器人 | 两轮改装 XLeRobot、双 SO-101 风格机械臂、云台头 |
| 传感器与音频 | D455、右腕 USB 相机、LD06 风格雷达、麦克风与扬声器 |
| Robot 主机 | Ubuntu 24.04 x86_64 · ROS 2 Jazzy |
| GPU 主机 | NVIDIA Ubuntu/WSL2；参考环境为 Ubuntu 24.04 WSL2 + RTX 3080 |
| VLM | LM Studio 提供 `qwen/qwen3-vl-4b` |

[BOM、接线与安装 →](docs/zh-CN/hardware.md) 右臂 Leader 仅在自行采集数据时需要。
本项目针对这一台参考构型，不是适用于所有原版 XLeRobot 的通用配置。

### 2 · Robot/GPU 双机安装

两台电脑克隆同一仓库，然后填写本机配置：

```bash
git clone --recurse-submodules https://github.com/xujiayuxian-png/xlerobot_home_service_demo.git
cd xlerobot_home_service_demo
cp config/local.example.yaml config/local.yaml
cp .env.example .env
```

设备路径、局域网地址和文件位置写在 `config/local.yaml`；两台 `.env` 使用相同 ACT token。
[前置依赖与配置清单 →](docs/zh-CN/install.md)

```bash
# GPU 主机
./tools/setup gpu
./tools/act download
# 如需原生 GPD 后端，将安装命令换为 ./tools/setup gpu --with-gpd

# Robot 主机：完整语音 + 寻人 Demo
./tools/setup robot --with-person-detector --with-kws-model --generate-voice-prompts
```

直接使用发布权重，**不必先自行数采或训练**。
其他推理资产见[模型与数据](docs/zh-CN/assets.md)。

<details>
<summary>可选模型的许可说明</summary>

Person detector 是显式启用的 AGPL-3.0 附加项。KWS 模型条款尚未明确；
安装工具直接从提供方下载，不随本仓库再分发。
启用前请看[安装说明](docs/zh-CN/install.md)和[第三方声明](THIRD_PARTY_NOTICES.md)。

</details>

### 3 · 检查依赖

```bash
./tools/doctor gpu     # 在 GPU 主机
./tools/doctor robot   # 在 Robot 主机
```

Doctor 只读检查，不打开相机或电机。首次缺少标定、地图或服务尚未启动是正常的；
完成下面的步骤后，在运行 Demo 前再次检查。

### 4 · 标定，并准备地图与地点

<p align="center">
  <img src="docs/images/robot-detail.png" width="100%" alt="标定概览与 XLeRobot 机械臂、头部的 URDF 细节可视化"/>
  <br/><sub>URDF 可视化 · <a href="docs/artwork/README.md">来源与渲染方法</a></sub>
</p>

```text
servo → head-camera → right-handeye ─┐
base（独立实测）────────────────────┴→ grasp-alignment → activate → render
```

[标定流程、打印标靶与回放 →](docs/zh-CN/calibration.md)

[统一标定网页：舵机 → 视觉 → 悬停验证 → 应用结果 →](docs/zh-CN/calibration-workbench.md)

[如何应用、替换和恢复标定配置 →](docs/zh-CN/calibration-versions.md)

| 头部相机自动标定 | 右臂手眼标定 | 悬停结果与建议补偿 |
| --- | --- | --- |
| [![头部相机页面](docs/images/calibration-head_camera.png)](docs/zh-CN/calibration-workbench.md) | [![右臂手眼页面](docs/images/calibration-right_handeye.png)](docs/zh-CN/calibration-workbench.md) | [![悬停结果页面](docs/images/calibration-hover.png)](docs/zh-CN/calibration-workbench.md) |

*真实前端的离线截图，使用示例数据，不代表实测精度。点击图片查看操作说明；[截图来源](docs/images/README.md)。*

随后[建图、保存 `table` 地点并激活场地](docs/zh-CN/mapping.md)。
你的机器人需要自己的标定与地图，仓库不附带其他设备的现场数据。

### 5 · 分别试试三个抓取后端

```bash
# GPU：先启动 LM Studio，再执行
./tools/run gpu

# Robot 终端 1：启动共用的 Demo 运行栈
./tools/run demo --hardware

# Robot 终端 2：每轮只提交其中一项任务
./tools/run grasp act --hardware
./tools/run grasp centroid --hardware
./tools/run grasp gpd --hardware
```

`grasp` 提交的是**完整取物递送任务**，不是仅移动机械臂的命令。
目标物品读取 `demo.object_id`。[后端行为说明 →](docs/zh-CN/grasping.md)

### 6 · 用语音或网页发起任务

Demo 启动后等待请求，不会自动开始任务。打开 **`http://<robot-host>:8080`**，
或说 **“小乐小乐”** 后发出取物指令。语音使用配置中的默认后端，网页可直接选择后端。

```text
定位 → 导航/贴桌 → 目标感知 → 抓取与验证
→ 寻人 → 接近 → 语音反馈 → 递送
```

[完整 Demo 操作说明 →](docs/zh-CN/demo.md)

## 配套工具，也可以单独使用

<table>
  <tr>
    <th>Demo 控制台</th>
    <th>建图与地点</th>
    <th>ACT 数采</th>
  </tr>
  <tr>
    <td width="33%"><a href="docs/zh-CN/demo.md"><img src="docs/images/demo-ui.png" alt="Demo 网页控制台的离线布局预览"/></a></td>
    <td width="33%"><a href="docs/zh-CN/mapping.md"><img src="docs/images/mapping-ui.png" alt="建图与地点工具的离线布局预览"/></a></td>
    <td width="33%"><a href="docs/zh-CN/act-workflow.md"><img src="docs/images/collection-ui.png" alt="ACT 数采工具的离线布局预览"/></a></td>
  </tr>
</table>

<p align="center"><sub>真实前端的离线布局预览，传感器留空；当前界面使用中文。</sub></p>

整个项目只保留五个入口：

| 入口 | 用途 |
| --- | --- |
| `tools/setup robot\|gpu` | 源码安装 |
| `tools/doctor robot\|gpu` | 检查依赖与服务 |
| `tools/calibrate` | 标定采集、求解、激活、回放和回滚 |
| `tools/act` | 数采、转换、训练、检查和下载 |
| `tools/run` | GPU 服务、建图、指定后端任务和完整 Demo |

自己的示教请走 [数采 → 转换 → 训练](docs/zh-CN/act-workflow.md)。
正常操作为 **Start → Home → End → 下一条**：有效数据默认保留，
End 后仍可遥操，方便放下物品并准备下一次采样。

## 范围、验证状态与许可

参考真机已完成以下功能流程验收：

- Demo、建图和 ACT 数采，使用既有生效标定。
- 从臂与头部舵机标定。
- 头部相机、手眼自动采集、求解与回 ready；手眼包含 20 个拟合点和 6 个留出验证点。
- 自动三点悬停测量、回 ready、结果可视化及建议补偿参数生成。

**流程通过不等于精度达标。** 悬停补偿尚未应用或复测，独立物理精度仍未验证；
新标定草稿不会自动替换 Demo 配置。
Leader 舵机标定已通过软件测试，尚未实机验收。
本轮未重新进行底盘/雷达和抓取对齐实验，参考 Demo 继续沿用既有参数。
测量范围与限制见[标定工作台说明](docs/zh-CN/calibration-workbench.md)；求解回放不代表实机精度验证。

真实动作必须显式使用 `--hardware`。本机配置、地图、标定、示教、权重和日志
不提交到 Git，运行资产通常保存在 `.xlerobot/`。

项目代码、文档与 ACT 权重：[Apache-2.0](LICENSE)。
30 条示教数据：[CC BY 4.0](https://huggingface.co/datasets/lissajous/xlerobot-glue-stick-grasp-30)。
[第三方资产保留各自条款。](THIRD_PARTY_NOTICES.md)

## 下一阶段：从 Demo 到养成系机器人

这个项目已经达到，甚至远超我最初的规划与期待。现在把 Demo、配套工具和示例整理开源，
希望大家可以参考、复现，也少走一些我走过的弯路。

继续使用这台改装 XLeRobot 时，三个结构上的限制越来越明显：

- **舵机差速轮的耐久性**：在我这台机器的使用中，耐久性不够理想。
- **作业高度固定**：机械臂安装高度不能升降，难以适应不同高度的作业面。
- **臂展与自由度有限**：够得到的范围、能采用的抓取姿态都受到限制。

这些是当前参考构型的使用体验，不是对所有 XLeRobot 的统一评测。
后续新增能力的主要开发将转向新的本体；本仓库保持当前 XLeRobot Demo 与复现工具的定位，
不把尚未验证的新本体配置混入现有安装流程。

近期计划参考 [AlohaMini](https://github.com/liyiteng/AlohaMini)，
出于成本考虑，搭建 **AlohaMini 二代底盘，暂时保留一代手臂方案**。
这是我的分阶段改造计划，不是已经完成或验证兼容的组合；
保留一代手臂也意味着臂展和自由度限制暂时仍在，底盘耐久性同样需要实际验证。

之后希望把更多精力放在 AI 上：从一件件具体家务开始，让机器人逐步熟悉家里的环境和习惯，
既能参与家务，也能提供陪伴，做一个和人一起慢慢成长的“养成系机器人”。
这是下一阶段的愿景，不是当前 Demo 已具备的能力。

<h2 align="center">关注作者</h2>

<p align="center">
  <strong>徐头头 · 小红书</strong><br/>
  <a href="docs/images/xiaohongshu.png"><img src="docs/images/xiaohongshu.png" width="360" alt="徐头头的小红书主页二维码名片，小红书号 LISSAGOGOGO"/></a><br/>
  <sub>打开小红书扫码，或搜索 <strong>LISSAGOGOGO</strong>；点击名片可查看原图。</sub>
</p>

---

[文档目录](docs/zh-CN/README.md) · [常见问题](docs/zh-CN/troubleshooting.md)
· [模型与数据](docs/zh-CN/assets.md) · [源码布局](docs/zh-CN/README.md#源码布局)
