# 常见问题

[文档目录](README.md) · [English](../en/troubleshooting.md)

优先看**第一条错误**和当次配置，后续 controller/lifecycle 错误可能只是连锁结果。
Doctor 检查配置、环境、路径、摘要和 HTTP 服务，不打开相机或电机设备。

| 现象 | 怎么处理 |
| --- | --- |
| 配置缺失或无效 | 复制 `config/local.example.yaml`，保留 `schema: xlerobot_demo/v1`；单机 ID 与数据集路径须一致。 |
| calibration/runtime 缺失 | 补齐五项后 activate、render、status，见[标定](calibration.md)。 |
| 缺地图或 `table` | 完成[建图 → 地点 → 验证 → 激活](mapping.md)，再填写两条 site 路径。 |
| LM Studio 不通 | 加载指定模型并开启局域网服务，从 Robot 检查 `/v1/models` 和 Windows/WSL 防火墙。 |
| 8765 启动失败 | `tools/doctor gpu` 检查传统环境与 SAM 2；GPD 额外需要 `tools/setup gpu --with-gpd`。 |
| centroid 可用但 GPD 失败 | 看具体候选/工作空间错误；GPD 本来就不会静默切成 centroid。 |
| 8766 启动失败 | 用 doctor 检查 CUDA、checkpoint、manifest 和 token。 |
| ACT unauthorized | 两台电脑 `.env` 使用相同 `XLEROBOT_ACT_TOKEN`。 |
| 公开模型无法下载 | 看[资产状态](assets.md)，等待公开发布不属于本机安装故障。 |
| 相机没画面或冻结 | 先停工作区再重插；D455 使用 USB 3，检查线材、带宽和腕相机稳定路径。 |
| 设备别名不存在 | 修复 udev 别名，不要换成不断变化的 `/dev/ttyUSB*` 序号。 |
| 语音听不到或不播报 | 检查输入/输出设备、KWS/Whisper 及本地生成的 prompts，见[安装](install.md)。 |
| 网页缺失或未更新 | 重跑 Robot setup 构建前端/工作区，然后重启并刷新浏览器。 |
| 真机命令被拒绝 | 准备好对应真机操作后，显式传入字面参数 `--hardware`。 |

## 数采

- **初始姿态提示：** 看具体关节、实测角度和允许范围。托住主从臂、释放扭矩、摆正、
  Reset、开始。该释放按钮不释放头部或左臂。
- **按 Home 后还在等待：** 准备到位时主臂保持扭矩，Home 后等 `RECORDING` 再示教，
  不要在 `STARTING_RECORDING` 时就拖动。
- **End 后仍然随动：** 正常设计，方便放下物品和归位；End 只停录制。
  需要停止随动并手动摆放时用释放扭矩。
- **没有通过按钮：** 新的完整采样默认保留；近期列表可拒绝或恢复，原始文件不删。
- **转换选不到数据：** 检查实际目录、改选状态和 `reviews/` 是否同步。
  已转换的 LeRobot 数据直接训练，不再执行原始转换。
- **随动突然停止：** 缺失/无效输入、反馈过期和遥操心跳超时会停止随动；
  排除错误后开始新一条。有限主臂读数会截断到从臂限位，正常碰到边界本身不应中止采集。
- **Reset/释放失败：** 不要强掰上力机械臂。停止数采运行环境，先检查第一条硬件/
  控制器错误再重启。

### Leader 时序提示

上游 `High execution jitter or mean error` 同时包含平均耗时和抖动两项检查，
不能只凭文案判断“抖动”。参考 Leader 以 50 Hz 读取六舵机，正常约 1.37 ms。
平均硬件执行耗时阈值为 2 ms 警告 / 4 ms 错误，标准差仍为 100 / 200 µs；
周期、反馈和遥操心跳检查保留。见
[Jazzy 诊断定义](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html#parameters)。
出现新的通信故障时不要不断放宽阈值掩盖问题。

## 停止与恢复

启动另一个工作区前，Ctrl-C 停止当前前台工具。
若退出报告扭矩释放失败或串口超时，安放好机械臂并关闭电机电源；
进程停止不等于舵机已经确认释放。排障时保留示教和标定草稿，不必删除重来。
