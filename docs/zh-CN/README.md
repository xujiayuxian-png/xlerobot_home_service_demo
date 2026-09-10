# 文档

[English](../en/README.md) · [项目概览](../../README.zh-CN.md)

## 跑现成 Demo

使用发布的 ACT 权重复现视频，**不需要 Leader、下载训练数据或自行训练**。

1. [硬件](hardware.md)：参考改装、接线和设备别名。
2. [安装与配置](install.md)：Robot/GPU 环境及本机设置。
3. [模型与数据](assets.md)：取得运行所需模型。
4. [标定](calibration.md)：通过 [统一网页工作台](calibration-workbench.md) 测量、验证和管理当前机器的标定。
5. [建图与地点](mapping.md)：保存地图及 `table`，验证并激活。
6. [完整 Demo](demo.md)：启动服务、检查就绪、语音或网页发起任务。
7. [比较抓取后端](grasping.md)：同一任务中切换 ACT、centroid、GPD。

## 采集并训练自己的抓取

机器人标定完成后，连接右臂 Leader，按 [ACT 数采、转换与训练](act-workflow.md)
操作。这条流程不需要在已建图的场地中导航。

两条路径的常见问题都放在[排障](troubleshooting.md)。
当前网页默认使用中文界面，主 Demo 和工具页面都有 **EN / 中文** 切换，浏览器会记住选择。
英文文档保留对应按钮名称。文档截图标注为离线界面预览，不作为机器人运行或验收证据。

[标定后续指南](calibration-followup.md)：底盘/雷达角度测量、可选抓取对齐、桌面悬停验证方案和标定版本替换。

[标定配置替换操作手册](calibration-versions.md)：首次应用、部分替换、版本切换与恢复。

## 源码布局

| 位置 | 内容 |
| --- | --- |
| `tools/` | 五个公开命令入口 |
| `config/local.example.yaml` | 本机配置模板 |
| `ros2_ws/src/` | ROS 包：共享任务、驱动、感知、操作、网页和工具 |
| `services/classical/` | GPU SAM 2 / GPD 感知提案服务 |
| `services/act/` | ACT HTTP 推理及训练流程 |
| `examples/calibration/` | 小型求解输入与预期回放结果 |
| `examples/grasping/` | 共享几何测试样本 |
| `assets/` | 打印标靶和模型/数据元信息，不存大权重 |
| `.xlerobot/`（忽略） | 本机地图、标定、示教、模型和日志 |

正常使用不需要逐个理解内部 ROS 包。
