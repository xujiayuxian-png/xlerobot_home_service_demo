# 单机标定

标定是本地资产，不是从参考机复制的一组常数。工具把不可变版本保存到
`.xlerobot/units/<unit>/versions/`，`active` 只指向已验证版本，并将 checksum 完整
的 active bundle 渲染到 `runtime/`。

按顺序完成：

1. 舵机零位、方向、raw 范围与关节范围。
2. 底盘轮径和轮距。
3. 固定 4 x 4 AprilTag 板的头部 D455 外参。
4. Tag 随夹爪移动的右臂 hand-eye。
5. 最终机械安装下的抓取对齐补偿。

可打印 PDF 和对应 SVG 源文件位于
[`assets/calibration_boards`](../../assets/calibration_boards/)。按 100% 比例打印
PDF、贴到刚性平面，并实测 tag 边长。

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
./tools/calibrate rollback --version <version>
./tools/calibrate render
```

无硬件求解回放：

```bash
./tools/calibrate replay head-camera
./tools/calibrate replay right-handeye
```

回放通过只证明求解器与数据合同正常，不代表你的机器人已完成标定。不要提交
`.xlerobot/`。
