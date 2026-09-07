# ACT：数采、转换与训练

[文档目录](README.md) · [English](../en/act-workflow.md)

这不是运行 Demo 的前置步骤；只想复现展示，直接用[发布权重](assets.md)即可。
[ACT 视频](https://www.bilibili.com/video/BV18RK66JEdP)展示了标定预抓取与学习接触阶段。

## 1. 在 Robot 主机启动数采

准备标定完成的机器人、D455、腕部相机、物品，以及
`robot.devices.leader_arm` 对应的单个右臂 Leader。先停止 Demo 或建图工作区。
数采不需要场地地图。

```bash
./tools/act collect --hardware
```

打开 `http://<robot-host>:8080`，端口以 `demo.web_port` 为准。
启动后头部用 3 秒转到水平居中、俯仰 **0.8 rad**，不因此移动主从臂或夹爪。
不同桌高可在启动时传 `--head-tilt RAD`，角度须在标定范围内；同一批数据保持视角一致。

![数采工作区：离线界面预览](../images/collection-ui.png)

*当前前端与示例标识的离线截图，未连接机器人，不包含相机数据。*

## 2. 采集一条数据

| 操作 | 会发生什么，以及需要等什么 |
| --- | --- |
| 选择数据集、物品和模板 | 修改物品自动填写抓取指令，也可手动改写。 |
| **开始 / 准备 pregrasp** | 抓取模板识别物体、打开从臂夹爪、主从臂并行准备。等到 `WAITING_HOME`，此时主臂保持扭矩，尚未录制。 |
| **Home / 释放主臂并开始采集** | 托住主臂后按 Home，等到 `RECORDING` 再拖动。开始随动与录制，同时释放主臂扭矩。 |
| **End / 结束并保存到本机** | 只停止录制并保存，仍可遥操放下物品和归位；这些后续动作不进入已保存数据。 |
| 再次 **开始** | 停止上一轮结束后的随动，再准备下一条；不覆盖已有数据。 |

**抓取模板**复用 Demo 的目标检测和预抓取，需要对应 VLM 服务；
**通用手动**跳过识别和从臂自动预抓取，只把主臂对齐从臂当前位置。
首次试录可选通用手动，时长设 10–15 秒。

Home/End 也支持键盘操作，输入框内和长按重复不触发。Home 后系统先保持扭矩，
取得录制首帧并检查对齐；看到 `RECORDING` 再开始示教，不是按下就立刻拖动。
等待 Home 超过 60 秒会取消，不会自动开始；取消或准备失败可能释放主臂扭矩，
请保持支撑。

达到时长上限等同正常 End。**Abort** 是异常中止，会保留 incomplete，
不要拿它代替保存。页面帧数来自实际录制器。
页面中的状态机 dry-run 不录制数据，也不表示硬件已经就绪。

## 3. 默认保留，不满意才拒绝

成功保存且完整的数据**默认保留**，不用每条点通过。
不想用于训练时点 **拒绝本条**，误拒绝可 **恢复保留**。
最近 20 条完成数据在下一条、刷新页面或重启后仍可改选；
旧的未审核数据需要手动保留。

改选只影响训练选择，不修改原始文件。自动保留标记为
`selection_source: automatic_on_save`，不冒充人工审核。数采不会上传数据。

## 4. 数据在哪、目录怎么配置

页面显示**机器人主机**上的保存路径和配置文件。
数据集 ID 是整批数据的名称；每个带时间戳的 episode ID 才是一次采样的名称。

```yaml
data:
  collection_root: .xlerobot/artifacts
  dataset_id: my-grasps
  dataset_root: .xlerobot/artifacts/datasets/my-grasps
  conversion_version: v1
  repo_id: local/my-grasps
```

```text
<collection_root>/datasets/<数据集 ID>/
├── raw/<episode ID>/   manifest.json、data.npz、videos/
├── reviews/           可修改的保留/拒绝记录
└── derived/<version>/lerobot/   转换结果
```

在 `config/local.yaml` 修改 `collection_root` 后重启数采；
`--config PATH` 可以指定其他本机配置。
页面更改数据集名称只影响后续采样。转换前让 `data.dataset_root` 指向完整数据集目录，
编辑配置时也保持 `data.dataset_id` 与目录名称一致。改配置不搬迁旧数据。

## 准备失败或随动停止时

托住主从臂 → **释放主从臂扭矩** → 等待确认 → 手动摆好 →
**Reset / 重置状态** → 开始。Reset 只清理会话，不删除数据、不回位、不上力，
下一次开始才准备。释放操作同时停用共用总线的底盘，但**不释放头部和左臂**。

初始姿态越界时，页面顶部会明确列出关节和允许范围；不要强行掰动已上力关节。
若涉及头部或左臂，需要停止运行并单独释放对应硬件。
释放或控制器状态无法确认时，先停止数采运行环境再恢复。
细节见[数采排障](troubleshooting.md#数采)。

## 5. 传输、转换与训练

将 `data.dataset_root` 指向包含不可变 `raw/` 和可改选 `reviews/` 的采集目录。先在
`config/local.yaml` 中填写 `transfer` 的 SSH 目标，再通过 Robot Web
UI 按需拒绝不想要的数据后，把整个数据集目录传到 GPU 的同一仓库相对路径。以示例配置为例（先替换
文档地址）：

```bash
# Robot 端；--ignore-existing 防止重跑时替换已有 episode。
rsync -a --checksum --ignore-existing --exclude reviews/ \
  .xlerobot/artifacts/datasets/xlerobot-glue-stick-grasp-30/ \
  operator@192.0.2.10:xlerobot_home_service_demo/.xlerobot/artifacts/datasets/xlerobot-glue-stick-grasp-30/
# 单独同步改选后的审核标签，不覆盖原始 episode。
rsync -a --checksum \
  .xlerobot/artifacts/datasets/xlerobot-glue-stick-grasp-30/reviews/ \
  operator@192.0.2.10:xlerobot_home_service_demo/.xlerobot/artifacts/datasets/xlerobot-glue-stick-grasp-30/reviews/
```

然后在 GPU 端执行：

```bash
./tools/act convert --dry-run
./tools/act convert
./tools/act train --dry-run
./tools/act train --output .xlerobot/outputs/xlerobot-act-local-grasp-v1
```

dry-run 会分别报告 accepted、rejected、缺 review 和缺 raw 的数量。转换直接复用
`xlerobot_dataset_tools`，只消费审核为 accepted 的不可变 episode，派生版本存在时
拒绝覆盖。训练命令固定 ACT、腕部
RGB `3 x 480 x 640`、六维 state/action、chunk 100、model dimension 512、FF 3200、
4 层 encoder/1 层 decoder、8 heads、latent 32、ResNet-18 ImageNet V1、VAE 开、AMP 关。

续训从输出目录的 `checkpoints/last` 恢复权重、优化器、随机状态及原训练配置。
数据集、batch size 和策略参数沿用原值，不要重复传入；`--steps` 是最终总步数，
不是额外训练步数：

```bash
./tools/act train --resume --output .xlerobot/outputs/xlerobot-act-local-grasp-v1 --steps 6000
```

中断后可省略 `--steps`，继续完成原定目标；已完成的训练需要更大的总步数。
仅有推理权重、没有训练状态的下载模型不能直接续训。

只验证软件流程时，用独立输出目录做短训练（产物不具备可用抓取效果）：

```bash
./tools/act train --output .xlerobot/outputs/act-smoke --steps 12 --batch-size 2
./tools/act train --resume --output .xlerobot/outputs/act-smoke --steps 16
./tools/act evaluate --checkpoint .xlerobot/outputs/act-smoke/checkpoints/000016/pretrained_model --output .xlerobot/outputs/act-smoke/qualification.json
```

验证过程中不要修改 Demo 正在使用的 checkpoint 配置。

发布模型只用 30 条黄色胶棒示教训练。计划完整公开的数据集为
`xujiayuxian-png/xlerobot-glue-stick-grasp-30`，许可 CC BY 4.0。羽毛球效果只作为
定性的 OOD 证据。

## 6. 检查 checkpoint 与下载

默认 5,000 steps 的训练会打印最终 checkpoint 的明确路径。对这个训练产物执行：

```bash
./tools/act evaluate \
  --checkpoint .xlerobot/outputs/xlerobot-act-local-grasp-v1/checkpoints/005000/pretrained_model \
  --output .xlerobot/outputs/xlerobot-act-local-grasp-v1/checkpoints/005000/pretrained_model/model-manifest.json
```

`evaluate` 加载本地 checkpoint、核对部署结构，并把来源信息及每个 checkpoint
文件的 SHA-256 写入 qualification manifest；它不测量真机成功率。把
`models.act_checkpoint` 与 `models.act_manifest` 分别指向该目录和 JSON。
`tools/run gpu` 同时接受这种本地 qualification schema 与发布后的固定下载 schema，
但启动服务前都会严格核对所列文件。

`download` 操作按 Apache-2.0 模型 ID
`xujiayuxian-png/xlerobot-act-local-grasp-v1`、immutable Hub revision 和全部固定
摘要下载；首次上传与 revision 写入 manifest 前有意不可用。发布后再使用公开入口
`tools/act download`。
