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
# 按预设姿态采集整块标定板，退出后执行终端打印的后续命令。

./tools/calibrate capture right-handeye --hardware
# 按预设姿态采集 Tag 23，退出后执行终端打印的后续命令。
```

每次只启动一个采集工作流，并打印本机 HMI 地址和准确的后续命令。启动页面本身不会
移动机器人；点击视觉标定姿态会产生真机运动。释放舵机扭矩前必须托住机械臂。原始
采集都留在 `.xlerobot/units/<unit>/capture/`，不会进入 Git。

已有采集不会被静默混入新结果。只有继续同一次头相机/手眼会话时才使用
`--resume`；机械安装变化后使用 `--fresh`，工具会把上一份工作流目录移入本地
`archive/`，不会删除数据。

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
