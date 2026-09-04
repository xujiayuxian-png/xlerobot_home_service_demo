# 参考硬件改造

本仓库只针对[公开取物递送视频](https://www.bilibili.com/video/BV1srNg6XEZj)
中的这一台双轮改装 XLeRobot；该视频也是公开的整机外观和安装位置参考。这里不承诺
兼容其他 XLeRobot 布局。

## 参考 BOM

下表只记录软件真正依赖的部件。原验证机没有保留下来的供应商 SKU、供电额定值和
购买链接不会凭空补写。

| 数量 | 参考部件 | 用途与复现说明 |
| ---: | --- | --- |
| 1 | 以 IKEA RÅSKOG 小推车为主体的 XLeRobot | 机械本体；仓库中的 description mesh 和坐标系与它对应。 |
| 2 | SO-101 风格 Follower 臂与夹爪 | 每侧五个关节加一个夹爪，使用 Feetech 串行舵机。 |
| 1 | Feetech 两自由度头部云台 | 与左臂共用左侧串行总线。 |
| 2 | Feetech STS3215 轮舵机与轮子 | 差速驱动，在右总线上的 ID 为 9、10。 |
| 2 | 相互独立的 Feetech USB 串口适配器 | 分别独占完整右总线和左总线。 |
| 1 | SO-101 风格右侧 Leader 臂与独立适配器 | 只用于 ACT 示教采集；推理和正常 Demo 不会打开它。 |
| 1 | Intel RealSense D455 | 头部 RGB-D 相机，供 VLM 定位和传统几何路线使用；应接 USB 3。 |
| 1 | Sonix `USB2.0_CAM1` 类 UVC 相机 | 固定在右腕，以 640 x 480 提供 ACT 观察；须使用稳定设备路径。 |
| 1 | LD06 类 2D 激光雷达 | 建图、定位和避障；参考机为倒置安装。 |
| 各 1 | 麦克风与扬声器 | 中文语音请求与反馈。 |
| 1 | x86-64 Robot 主机 | Ubuntu 24.04 + ROS 2 Jazzy；独占所有运动设备。 |
| 1 | 局域网 NVIDIA 主机 | Ubuntu 24.04 或 WSL2；验证过的推理机为 RTX 3080。 |

支架、线束、供电分配、急停硬件、紧固件及小推车改装都属于具体机器的机械工程。
请按自己的负载和电源重新设计并校验，不要从软件仓库反推额定参数。

## 总线与传感器拓扑

```text
Robot 主机
├── /dev/right_arm       Feetech 总线：右臂/夹爪 ID 1-6
│                                      轮子 ID 9-10
├── /dev/left_arm        Feetech 总线：左臂/夹爪 ID 1-6
│                                      头部 pan/tilt ID 7-8
├── /dev/right_master_arm  右侧 Leader ID 1-6（仅采集）
├── /dev/lidar           倒置 LD06 类雷达
├── USB 3                头部 RealSense D455
├── 稳定 /dev/v4l/...    右腕 USB2.0_CAM1
└── 音频输入/输出        麦克风与扬声器

局域网 GPU 主机
├── LM Studio / Qwen                 :1234
├── SAM2 + 可选 GPD 候选服务         :8765
└── ACT 推理                         :8766
```

舵机 ID 只要求在各自物理总线内唯一；三条独立总线都使用 ID 1-6 是有意设计。每条
物理总线只能有一个运行时 owner，不能再启动另一套 arm/base driver 抢同一适配器。

仓库内 ROS description 是连杆几何、关节名和总线映射的机器可读参考。其中的数值
描述的是视频里的验证机，不是可以直接复制到新机器的标定文件。

## 安装假设

- D455 位于 pan/tilt 链下方并发布经标定的头相机坐标系；应保持视向及 USB 3 带宽。
- 腕相机刚性固定在右腕，其位姿和画面方向都是学习策略 observation contract 的一部分。
- 参考雷达变换包含 180° roll。改变安装方式后必须修改 description 并重新验证地图。
- 每台机器都要用底盘标定估计轮半径和轮距，不能把参考 YAML 数值当成实测值照抄。
- 头相机、手眼和抓取对齐共同闭合相机支架与机械臂几何；外观看起来相同也要生成
  自己的完整 bundle。

## 接线与稳定设备名

1. 更改串行总线前先断电，保证同一总线的舵机类型、电压和适配器与实际装配兼容。
2. 右臂、右夹爪和两只轮子接右适配器；左臂、左夹爪和头部接左适配器；Leader
   始终使用自己的适配器。
3. 加扭矩前扫描每条总线，核对上述精确 ID，并保证该总线内无重复 ID。
4. 按 `config/local.yaml` 建立 `/dev/lidar`、`/dev/right_arm`、
   `/dev/left_arm`、`/dev/right_master_arm` 稳定 udev 别名。
5. 多台 RealSense 时填写 D455 serial；腕相机必须使用 `/dev/v4l/by-id` 或
   `/dev/v4l/by-path`，不要使用会漂移的 `/dev/videoN`。
6. 在头部和机械臂全行程内检查线缆无拉扯、无夹点；D455 应保持 USB 3，不与腕部
   UVC 相机争用受限链路。
7. 在 `config/local.yaml` 中确认麦克风和扬声器；ALSA card 序号不能跨机器照抄。

## 首次上机边界

先运行 `./tools/doctor robot` 并完成全部标定，再允许运动。操作员应留在机器人旁，
清空工作区，首次方向检查时把驱动轮架空，并保证物理停止手段触手可及。本仓库不会
远程替你验证这些条件。

标定完成后，建图和地图验证也是显式真机操作：

```bash
./tools/run mapping --hardware --phase build
./tools/run mapping --hardware --phase validate
```

缺少字面参数 `--hardware` 时会在打开电机设备之前失败；抓取和完整 Demo 同样如此。
