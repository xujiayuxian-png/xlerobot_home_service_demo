# 犀牛派 X1 原生 Ubuntu 22.04 适配

[English](../en/x1-controller-migration.md) · [文档目录](README.md)

状态：**P1 原生安装、编译与软件验收通过；尚未通过实机运动验收**。
更新日期：2026-09-12。此前的容器/Jazzy建议已被本次用户指定的原生方案替代。

本机实测结果：

| 检查 | 结果 |
| --- | --- |
| 正式 setup 入口 | 原生安装完成，20 个 ROS 包全部构建成功 |
| ROS 软件回归 | 全工作区测试及修复项复验后，当前 colcon 汇总 1021 项，0 errors、0 failures、45 skipped |
| HMI 前端 | 13 个测试文件、96 项通过，生产构建成功 |
| 标定与数采 | 标定 161 项、数采 58 项通过；新增悬停图像检测另测 12 项通过 |
| 无设备运行 | 真实 Nav2 控制器、碰撞监测、规划器和行为树激活通过；模拟总线与 Leader 生命周期通过 |
| 本地推理 | CPU 寻人推理通过，KWS 与 Whisper int8 模型加载通过；22 个语音提示已生成 |
| 公开工具 | 五入口 help、shellcheck、无 --hardware 的 Demo 拒绝启动检查通过 |
| doctor | ROS / 依赖 / 模型通过；18 errors、1 warning 均为待接入设备、标定/地图、GPU 地址与 token 等运行条件 |

版本记录和验收日志在本机忽略的 `.xlerobot/environment/`；软件测试使用模拟 I/O，
不是相机采集、麦克风识别、50 Hz 实机时序或完整 Demo 的通过证据。

## 1. 本次环境与职责

X1 使用原厂 AidLux / Ubuntu 22.04 ARM64、Python 3.10 和 ROS 2 Humble。
不安装容器，不混装 Noble 软件包，不升级厂商内核或刷机。
[Humble 官方平台说明](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html)
列出 Ubuntu 22.04 的 ARM64 支持。原 Ubuntu 24.04 x86_64 / Jazzy 参考环境继续保留。

X1 承担驱动、ros2_control、本地 ACT executor、MoveIt、导航、任务、网页、语音、寻人、
标定与数采。GPU 主机继续运行 LM Studio、SAM 2/GPD、ACT 推理和训练，接口保持原样。
所有远程服务只返回数据，控制器命令仍在 X1 生成。

本机为 Qualcomm QCS8550，6 核、约 14 GiB 内存；初始构建按单包顺序、包内 2 线程执行。
50 Hz 控制配置和 watchdog 保持原值；这并不证明厂商 PREEMPT 内核已满足控制时序要求。

## 2. 原生安装

源码必须包含固定的 SCServo 子模块；不迁移 x86 的 build、install、venv 或缓存。
在仓库根目录运行：

```bash
# 仅安装 apt / ROS / rosdep 依赖；需要 sudo。
./tools/setup robot --system-only

# 安装本机 Python 依赖、下载模型、构建 HMI 和全部 ROS 包。
./tools/setup robot --with-person-detector --with-kws-model --generate-voice-prompts
```

新工作副本还需从 `config/local.example.yaml` 创建忽略的 `config/local.yaml`，
设置 `demo.web_port: 18080` 并填写实际设备和 GPU 地址；秘密单独放入 `.env`。

setup 自动识别 `Ubuntu 22.04 + aarch64`，使用 `/opt/ros/humble`。
GPU 仍按原 Ubuntu 24.04 x86_64 约束检查。安装不会启动机器人节点或打开电机设备。
ROS apt 源限定 Jammy/ARM64，校验 Open Robotics 签名密钥；不关闭包签名验证。
ROS 官方 apt 地址使用 HTTP，完整性由签名索引和包哈希验证。

Node 22.22.0 下载到忽略的 `.xlerobot/vendor`，校验官方 SHA-256 后使用；
Jammy 自带 Node 12 无法构建当前 Vite 前端。Python 环境位于 `.venv/robot`，
colcon 使用该解释器生成 Python 节点入口，避免节点遗漏语音等 venv 依赖。
`XLEROBOT_BUILD_WORKERS` 可设置包内并发，默认 2，初次验收前不要盲目提高。

X1 依赖固定在 `requirements/robot-x1.txt`：ONNX Runtime 1.23.2 支持 Python 3.10，
NumPy 1.26.4 与 OpenCV 4.11.0.86 保持 Humble cv_bridge 的 NumPy 1.x ABI。
寻人使用 `robot-person-x1-agpl.txt` 中的 ARM64 PyTorch CPU wheel；
不能直接改用 PyPI 的同版本 torch，它会引入 CUDA 依赖。
原参考环境的依赖锁定不变，实际环境清单写入忽略的 `.xlerobot/environment`。

寻人使用 AGPL Ultralytics；KWS 上游模型条款未明确，下载选项明确保留。
Whisper、KWS 和寻人模型均按仓库清单校验，模型不进入 Git。

## 3. 本机包管理修复记录

首次检查发现 `libkmod2 29-1ubuntu1.1` 已解包但未配置，已安装的 `kmod` 为 `29-1ubuntu1`。
常规修复试图更新 kmod，却与 AidLux `mod-blacklist` 所有的
`/etc/modprobe.d/blacklist.conf` 冲突。此次将 libkmod2 恢复为匹配的 `29-1ubuntu1`，
保留厂商 blacklist 文件；未强制覆盖、移除厂商包或修改内核。
这是本机已有包状态的修复记录，**不写入通用 setup 自动执行**。
以后若再次出现此冲突，应先检查 `dpkg --audit` 和 `apt-get -s --fix-broken install`，
不要直接使用 `--force-overwrite`。

## 4. Humble 兼容边界

- 硬件插件兼容 Humble 的 HardwareInfo 初始化接口；总线门控、标定、扭矩和读写实现共用。
- 标定检测器兼容新版 OpenCV 的 ArucoDetector，并在旧单 Tag 位姿接口缺失时用相同角点坐标
  与迭代 solvePnP 求解；合成数据测试核对旋转、平移和重投影误差。
- Leader 控制器兼容 Humble 的同步接口写入；心跳过期及生命周期退出仍关闭扭矩。
- 所有公开入口按本机平台选择 ROS；测试通过包索引寻找 controller_manager，不写死 Jazzy 路径。
- Leader 租约参数通过 spawner 的参数文件传递，支持 Humble 与 Jazzy。
- Humble 的 SLAM 是普通节点，没有新版在线 Reset 服务。建图、保存仍可用；清空现场地图
  需要停止并重新启动建图会话。网页会明确拒绝不支持的在线重置，保留预览和已保存资产，
  不把清除交互编辑的 Clear 服务冒充地图重置。
- Nav2 在 Humble 加载小型参数覆盖和 BT.CPP 3 行为树，兼容插件名、progress checker 和碰撞点数参数。
- Humble 的 Nav2 action 没有 Jazzy 的细分错误码。仍同时检查 action 状态和非空结果，
  失败不会变为成功；BackUp 中止后不再前进重试。上层收到通用后端错误，无法声称已识别具体碰撞原因。
  本地取消等待和超时保留。Humble 的恢复树不使用 Jazzy 的错误码条件，实际恢复行为需在 P3/P4 复验。

Humble 中不存在的 Jazzy 控制器诊断参数不能代替实测。需要另行测量 50 Hz 周期的 p95/p99、
CPU、温度、串口耗时与超时，逐项叠加相机、语音、寻人和网页负载。

## 5. 本地配置与资产

机器配置只放在忽略的 `config/local.yaml`；秘密只放在忽略的 `.env`。
本次创建的本地配置设置 `demo.web_port: 18080`。AidLux 已使用 8080，不停止 filebrowser。
标定工作台另用 `--web-port 18080`，Demo、标定、建图、数采交替运行，避免争抢设备。

接线前核对：

- GPU 可达地址、X1 到 GPU 的 SSH、公用 ACT token；示例地址不可直接使用。
- D455 USB3 直连、实际序列号、腕相机和串口稳定别名、音频设备及用户组权限。
- 同一机器人的完整 active 标定版本、地图和地点。不要复制笔记本的数字设备序号或绝对路径。
- D455 初始保持 RGB/depth 640×480 @ 15 fps、对齐深度、不启用点云和 IMU。

同机且机械安装未变时可私下迁移已有标定，不自动重做物理标定。
迁入后执行标定 status/render 并检查实际版本和校验值，参考[标定版本说明](calibration-versions.md)。
若相机或机械安装改变，重测受影响项目。

## 6. 验收顺序

```bash
# 软件检查：不连接电机设备。
./tools/setup --help
./tools/doctor --help
./tools/calibrate --help
./tools/act --help
./tools/run --help
source /opt/ros/humble/setup.bash
source .venv/robot/bin/activate
cd ros2_ws
source install/setup.bash
python "$(command -v colcon)" test --executor sequential --event-handlers console_cohesion+
colcon test-result --verbose
cd ..
./tools/doctor robot
```

安装初期 doctor 对缺少实机标定、地图、设备和 GPU 服务的错误表示剩余接入工作，
不能据此声称软件编译失败，也不能忽略这些错误启动真实 Demo。

| 阶段 | 完成条件 |
| --- | --- |
| P1 原生环境 | X1 全部源码构建、HMI 测试/构建、软件测试及五入口检查 |
| P2 外设 | 接线后验证相机、雷达、音频、GPU HTTP；目标连续采集 30 分钟，记录实际结果 |
| P3 控制 | 明确授权当次运动测试后，逐项测底盘、头部、双臂、夹爪、Leader 与 ACT executor；测时序 |
| P4 Demo | ACT 默认八阶段流程，另测 centroid/GPD、语音/HMI；笔记本不运行任何控制节点 |
| P5 配套 | 建图、数采、转换、GPU 训练调度、标定与版本替换 |

所有真实运动仍要求公开命令显式携带 `--hardware`，软件安装授权不等于运动测试授权。
硬件验收前不将 X1 写成完整 Demo 已验证环境。保留旧笔记本回退环境；切换主控前停止另一台的节点。
NPU、本体改造和端侧 VLM 仍留作独立后续任务。

当前 X1 的按需图像订阅、固定语音、手动服务启停与 CPU 对照结果见
[X1 Demo 降负载实施与验证](x1-cpu-optimization.md)。静态对照结果与完整实机验收分开记录。
