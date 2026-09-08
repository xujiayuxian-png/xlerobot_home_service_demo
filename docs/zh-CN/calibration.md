# 单机标定

[文档目录](README.md) · [English](../en/calibration.md)

## 准备与顺序

已有可用标定时，先保留它为 active，新测量只写 draft。
“采样保存”“求解成草稿”和“激活成运行标定”是三件不同的事。

| 步骤 | 准备或测量什么 | 完成时应该看到什么 |
| --- | --- | --- |
| 舵机 | 总线 ID、关节方向、零位与 raw 行程 | 关节集合、范围和限位校验通过 |
| 头相机 | 刚性固定的 36h11 4×4 板，ID 0–15，tag 边长 40 mm | 不同头部姿态采样，求解并保存有效 draft |
| 右臂手眼 | 夹爪上固定 36h11 Tag 23，边长 60 mm，由 D455 观察 | 不同机械臂姿态采样，求解并保存有效 draft |
| 底盘 | 用卷尺等测量直行距离与旋转角度 | 轮径/轮距拟合通过 |
| 抓取对齐 | 同一点的视觉/FK 坐标，以及单独测量的静止下垂量 | 视觉补偿和重力下垂分别保存 |
| 激活 / render | 同一机器的五项完整结果 | `status` 显示预期 active 版本与有效 runtime |

依赖链为 `servo → head-camera → right-handeye`。底盘实测可独立进行；
四项结构标定齐全后做抓取对齐。每项开始前，停止其他 Demo、建图或数采工作区。

打印 [头相机标定板 PDF](../../assets/calibration_boards/head_4x4_ids_0-15_40mm.pdf) 和
[Tag 23 PDF](../../assets/calibration_boards/handeye_tag23_60mm.pdf)，比例 100%。
头部板 tag 间距 12 mm；打印后用尺确认真实尺寸，不使用截图打印。

## 采集并保存草稿

先把 `config/local.example.yaml` 复制到 Git 忽略的 `config/local.yaml`，填写单机 ID
和稳定设备路径。真机采集全部经过同一个公开入口，并要求显式确认硬件：

```bash
./tools/calibrate capture servo --hardware
# 在网页中完成并 finalize，Ctrl-C 退出，再执行终端打印的后续命令。

./tools/calibrate capture head-camera --hardware
# 网页预览标定板，再一键采集、求解并保存 head_camera 草稿。

./tools/calibrate capture right-handeye --hardware
# 按预设姿态采集 Tag 23，退出后执行终端打印的后续命令。
```

每次只启动一个采集工作流，并打印本机 HMI 地址和准确的后续命令。
舵机页启动仅连接串口，不使能扭矩或发送运动指令。头相机工作区只连接头部所在总线、
使能头部两个舵机的保持扭矩；不操作双臂、夹爪或轮子。手眼工作区启动完整运行栈的
保持扭矩，但不自动回 ready。点击姿态或开始自动采集后才运动。释放舵机扭矩前必须托住机械臂。原始
采集都留在 `.xlerobot/units/<unit>/capture/`，不会进入 Git。

舵机未完成会话会自动恢复，重启后暂停等待你继续；网页刷新不会重置采样。
只有继续同一次头相机/手眼会话时才使用
`--resume`；机械安装变化后使用 `--fresh`，工具会把上一份工作流目录移入本地
`archive/`，不会删除数据。

### 舵机网页：整组活动，一次完成

按 **右臂 → 左臂 → 头部** 三组操作，不用选择单个关节或反复点开始/结束。
这里的两臂是机器人上的 Follower，不包含数采用的 Leader。Leader 目前尚无对应的
整组网页标定工具；不要直接把 Follower 的零位数据用于另一条 Leader 臂。

1. 选一组，托住后点击“释放本组扭矩”。其他组和轮子不受影响。
2. 选择零位来源：
   - **同一台机器未拆装舵机、未修改硬件零偏：** 可明确选择保留当前有效标定的零位，
     只重新采集行程。页面显示来源；不会自动把旧行程当成新测量。
   - **新装或确实要重测零位：** 参考页面的 URDF 零位图摆好整组，再记录一次零位。
     机械零位不是 ready，也不是任意舒服的中间姿态。
3. 点击整组采集，自由活动这一组的各个关节（包括夹爪）。所有关节同时记录；
   实时位置、最小/最大值和覆盖条会告诉你哪些还没活动够。
4. 点击“完成本组”。不足的关节会明确提示，继续补采即可，已有范围不清空。
   每个关节仍需达到既有的 60% 范围要求，不必强顶机械限位。
5. 三组完成后保存全部结果，再按终端打印的命令导入严格校验后的 draft。

“暂停/继续”保留进度；“重置本组”只清除明确选中的采集草稿，不改舵机 EEPROM
或旧 active 标定。采集状态保存在本机同目录的 `session.yaml`，采集中约每秒更新，
异常断电最多可能丢失最后约一秒。进程重启后需要重新确认释放扭矩，才继续手动采集。
最终 `result.yaml` 只在全部通过时生成；保存不会自动恢复扭矩或激活运行标定。

交互参考 [LeRobot v0.5.1 的整臂采集](https://github.com/huggingface/lerobot/blob/1396b9fab7aecddd10006c33c47a487ffdcb54b4/src/lerobot/robots/so_follower/so_follower.py)。
本项目保留 ROS 的物理零位映射，不调用 LeRobot 的电机 homing 写入，也不把腕部
直接假定成 0–4095 全行程。编码器跨边界或读取失败会显示为问题，不算有效覆盖。

### 头相机网页：预览后，一键采集与求解

先导入通过校验的舵机草稿，再启动 `./tools/calibrate capture head-camera --hardware`。
将整块 4×4 板平放并固定在桌面；无需测量板相对底盘的位姿或桌面高度，求解器会同时估计板位姿。

1. 在页面点击“预览位置”，头部移动到 pan=0、tilt=0.8 rad。确认图像中整板清晰可见，
   检测状态为 16 个 Tag、重投影误差小于 1 px。
2. 确认本次头部运动并开始自动标定。头部按同一份配置走 25 个姿态，
   范围为 pan −0.30～0.30 rad、tilt 0.60～1.00 rad；每次到位、等待稳定后再采样。
3. 页面显示标记叠加画面、识别状态、当前姿态及有效样本数。看不到完整板的姿态会明确标记为跳过，
   不拿重复样本凑数。至少 12 个独立样本且姿态覆盖、残差均通过，才会保存 head_camera 草稿。
4. 完成后查看平移 RMS/p95、旋转 RMS 和结果路径。此时没有激活新标定，旧 Demo runtime 不变。

序列在机器人上运行，刷新网页不会重启动作。“暂停”取消当前轨迹并保留采样，之后可继续；
进程重启也不会自行运动，同一会话通过 `--resume` 重新打开。不要在采样期间移动板或底盘。
如果移动了任意一个、重新做了舵机标定，或者要完整重测，请退出并用 `--fresh` 归档旧采集再开始，
不能把两种摆放下的数据混在一起。

自动求解与命令行复用同一个严格求解器。质量未过会显示原因并保留原始样本，不生成“通过”的草稿，
也不降低阈值。首次装配机器仍需后续手眼、底盘和抓取对齐，才能激活完整 bundle。

### 其他分项与草稿导入

底盘几何采用卷尺实测。可以填写 `examples/calibration/base_measurements.yaml`，也可以
运行 `./tools/calibrate capture base --hardware` 使用网页表单；表单不会驱动底盘，
直行和旋转试验需在现场监护下执行，再填入命令值与实测值。

随后由公开严格求解器/校验器保存各项 draft；采集页面不会激活 bundle。下面使用
示例单机 ID `demo-01`，若修改了 `robot.unit_id`，请对应替换：

```bash
./tools/calibrate servo --input .xlerobot/units/demo-01/capture/calibration_work/servo/result.yaml
./tools/calibrate base --measurements /path/to/base_measurements.yaml
# 或导入底盘采集页面打印的结果：
./tools/calibrate base --input .xlerobot/units/demo-01/capture/calibration_work/base_geometry/result.yaml
./tools/calibrate head-camera --samples .xlerobot/units/demo-01/capture/calibration_work/head_camera/samples.yaml
./tools/calibrate right-handeye --samples .xlerobot/units/demo-01/capture/calibration_work/right_handeye/samples.yaml
./tools/calibrate grasp-alignment --measurements /path/to/alignment.yaml
./tools/calibrate status
```

后续采集必须读取同一台机器已经通过的前序 draft，但 incomplete bundle 不能激活成
Demo runtime。用下面的 staged render 合成采集输入：

```bash
./tools/calibrate render --for head-camera      # 需要 servo
./tools/calibrate render --for right-handeye    # 需要 servo + head-camera
./tools/calibrate render --for grasp-alignment  # 需要四项结构标定
```

底盘拟合只读取 commanded/actual 卷尺测量，可与 `servo → head-camera →
right-handeye` 链路并行；到 grasp-alignment staged render 和最终发布时才成为必需项。

`tools/calibrate capture` 会自动执行 staged render。输出目录为
`.xlerobot/units/<unit>/draft/runtime/<workflow>/`，manifest 明确标记
`source: draft` 和 `final_demo_runtime: false`；Demo 绝不会读取这个 staging 目录。

所有分项通过后：

```bash
./tools/calibrate activate
./tools/calibrate render
./tools/calibrate status
```

不带 `--for` 的 `render` 只读取完整 active bundle，绝不会把 staged draft 提升为
最终的同级 `runtime/` 目录。

`tools/run demo --hardware` 只读取这些已渲染文件；缺失或损坏时会拒绝启动，不会
直接读取分项草稿。回滚使用：

```bash
./tools/calibrate rollback --version VERSION_ID
./tools/calibrate render
```

把 `VERSION_ID` 替换为要恢复的实际已保存版本。

同一台机器人已有可用标定时，可以导入完整运行快照并保留原数值和来源。
输入目录需包含 `geometry.yaml`、`servos.yaml`、`controllers.yaml`、
`transforms.yaml`、`grasp_alignment.yaml`；后两者记录 provenance，抓取对齐
标记 `validation: existing_unit_runtime`。导入会检查数值和文件完整性，
不代表旧测量已通过新的求解质量门。只适用于同一实物且安装未改变的机器人；
新装机器人按前面的测量流程操作。

```bash
./tools/calibrate import-runtime --input PATH --version existing-unit-v1
```

无硬件求解回放：

```bash
./tools/calibrate replay head-camera
./tools/calibrate replay right-handeye
```

回放通过只证明求解器与数据合同正常，不代表你的机器人已完成标定。不要提交
`.xlerobot/`。

## 底盘与抓取对齐怎么测

[底盘测量模板](../../examples/calibration/base_measurements.yaml)中的数字只是格式示例。
把名义轮径/轮距、命令距离/角度和实际测量值替换成自己的记录。
底盘页面只是填写表单，不会替你执行直行或旋转试验。

[抓取对齐模板](../../examples/calibration/grasp_alignment_measurements.yaml)
目前没有现场采集页面，需要在最终安装和头部姿态下准备本机 YAML，至少三组：

- `vision_xyz_m`：视觉测量点，在 `base_link` 中，以米为单位。
- `fk_xyz_m`：对应同一点的机器人 FK 坐标，使用相同坐标系。
- `settled_z_shortfall_m`：单独实测的静止向下偏差，为非负距离。
- `head_pose_rad`、`workspace_m`：真实头部姿态和测量区域。

不要把同一份下垂误差同时算进坐标补偿和单独下垂量。
求解器分别拟合 `FK − vision` 的平均补偿和平均下垂。
另外留出未参与拟合的点，用来检查对齐效果。

## 怎么看求解结果

头相机至少 12 组，手眼至少 20 组；反复采同一姿态不够，
还要满足姿态覆盖度和可观测性检查。保持 tag 可见，同时改变位置和朝向。

手眼质量门为平移 RMS 小于 10 mm、p95 小于 15 mm、旋转 RMS 小于 5°。
最大误差单独报告，不用它作为唯一拒绝依据。
完整阈值见 [quality.yaml](../../ros2_ws/src/xlerobot_calibration_tools/config/quality.yaml)。
拟合残差小，不等于实机对齐已经验证。

激活新版本后，用未参与拟合的点和一次受控抓取检查效果，
再把它视为旧可用标定的替代版本。回放通过不代表实机精度验收。
