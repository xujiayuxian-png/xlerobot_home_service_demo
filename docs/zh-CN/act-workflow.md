# ACT 数据到模型流程

标定预抓取与学习接触阶段可见
[两阶段 ACT 视频](https://www.bilibili.com/video/BV18RK66JEdP)。

五个公开 ACT 操作组成一条可审计路径：

```text
Robot 采集 → GPU 转换 → GPU 训练 → checkpoint 资格检查
                                     |
                         发布后按 manifest 固定下载
```

## 采集

异常恢复：托住主从臂 → 点击 **释放主从臂扭矩** → 等待成功提示 → 手动摆好
→ **Reset / 重置状态** → **开始**。当前录制会先中止并保留 incomplete，再释放扭矩。
右臂与底盘共用总线，因此底盘一并停用，头部保持。
Reset 会在主臂驱动退出后重新配置主臂总线及控制器，等待新的六关节反馈；主臂轨迹控制器仍停用。
释放扭矩使用硬件生命周期接口确认关闭，不能把停用控制器的服务成功回复当成舵机确认。
Reset 清理会话显示，不删除数据、不上力、不自动回位；下一次点击开始才从从臂实测姿态重新接管并准备，底盘仍停用。
若子动作停止无法确认，会报告具体问题，需重启采集运行环境，不会直接清掉未知的控制权。

接好 `robot.devices.leader_arm` 指定的单个右臂 Leader，并渲染 active 标定后，在
Robot 端运行：

```bash
./tools/act collect --hardware
```

启动后头部会用 3 秒转到水平居中、俯仰 **0.8 rad**，让 D455 看向桌面；
不会因此移动主从臂或夹爪。不同桌高可在启动时覆盖，角度必须在 active 标定限位内：

```bash
./tools/act collect --hardware --head-tilt 0.8
```

同一批数据保持视角一致，录制期间不调整头部。

它会启动源码采集工作区，右侧 Follower 机械臂可能运动。通过 Web UI 审核每条不可变
episode；原始录制不进入 Git。

打开 `http://<Robot 地址>:8080`（端口以 `demo.web_port` 配置为准）。启动页面不会
自动开始示教。初始姿态超出命令限位时，驱动仍启动并报告真实位置，但对应总线
保持不上力、不执行命令；手动调整到范围内后保持当前位置恢复，不执行积压目标。
开始示教前，主从臂姿态仍需处于允许范围内。

主臂的 URDF 和硬件命令范围统一由附件标定的 `raw_min/raw_max`、offset、direction
换算，沿用原型的物理范围；不使用该旧文件中从臂式的规划角度范围作为主臂启动门槛。
随动沿用原型的一对一关节映射，输出截断到当前 active 从臂标定的运动范围。
主臂到达或略超从臂边界时，从臂保持在边界，不中止采集；主臂回到范围内即继续跟随。
夹爪闭合读数略小于零也按这一规则处理，不改变标定值。启动不自动回位或开始录制。
主臂空闲及遥操期间仅反馈位置，位置轨迹控制器保持 `inactive`；点击开始后检查
实测姿态和控制器状态，再从当前姿态接管控制。越界时页面仍可启动，但准备动作会
明确列出需调整的关节，不自动放宽限位或带着旧目标重新上力。

1. 首次试录选择 **通用手动**，时长设为 10–15 秒。它以从臂当前姿态为起点，
   不进行目标检测或自动预抓取；**抓取模板** 则先识别目标、把从臂移到目标上方，
   复用 Demo 的预抓取流程。
2. 确认数据集 ID，修改物体名会自动生成“抓住＋物体名”指令，也可手动改写指令。
   试录使用独立数据集，不混入正式示教。
3. 点击 **开始 / 准备 pregrasp**。抓取模板复用同一份经过验证的预抓取目标，
   打开从臂夹爪后，主从臂并行准备；通用手动仅把主臂对齐从臂当前姿态。
   到位后显示 `WAITING_HOME`，主臂保持扭矩，此时尚未录制，不要拖动主臂。
4. 托住主臂，点击 **Home / 释放主臂并开始采集**（也可按键盘 Home）；
   `STARTING_RECORDING` 时仍保持主臂扭矩，先确认录像首帧和双臂对齐，再退出主臂
   位置控制、开启随动并释放主臂。显示 `RECORDING` 后拖动主臂，
   右侧从臂随动。Home 之前不释放扭矩、不记录示教；等待超过 60 秒会中止本条。
   注意：准备失败、取消或超时会退出流程并释放主臂扭矩，不等同于到位等待。
   若提示主臂未到位，错误信息会列出具体关节的目标、实测值和误差；不要强行开始遥操。
5. 完成后点击 **End / 结束并保存到本机**（也可按键盘 End）；达到最长时长也会正常结束。
   系统先停止随动并确认从臂停止，再保存双相机视频和关节数据，不会上传。
6. 等待保存成功，选择 **接受** 或 **拒绝**。只有接受的 episode 进入转换；拒绝
   不删除原始文件。再次点击“开始”会准备新的一条，不覆盖上一条。

**Abort** 用于中止异常采集，会保留 incomplete，不要用它代替正常结束。
**状态机 dry-run** 只验证状态流程，不录制数据，也不证明相机或机械臂已准备好。
Home/End 快捷键仅在对应阶段生效，输入框内和长按重复不触发。页面帧数来自实际
录制器，不是主臂反馈消息数。遥操发生数据缺失、非有限值、反馈丢失或心跳超时后，本条中止，
不会随反馈恢复而自动继续；排除原因后重新点击开始。若控制器状态无法确认，需重启
采集工具；重启不会自动启动新一条。
原始数据位于 `data.collection_root/datasets/<页面中的数据集 ID>/raw/`，
审核记录位于同级 `reviews/`；后续转换时让 `data.dataset_root` 指向该数据集。

## 转换与训练

采集端会在 `data.dataset_root` 下写入 `raw/` 和不可变 `reviews/`。先在
`config/local.yaml` 中填写 `transfer` 的 SSH 目标，再通过 Robot Web
UI 完成审核后，把整个数据集目录传到 GPU 的同一仓库相对路径。以示例配置为例（先替换
文档地址）：

```bash
# Robot 端；--ignore-existing 防止重跑时替换已有 episode。
rsync -a --checksum --ignore-existing \
  .xlerobot/artifacts/datasets/xlerobot-glue-stick-grasp-30/ \
  operator@192.0.2.10:xlerobot_home_service_demo/.xlerobot/artifacts/datasets/xlerobot-glue-stick-grasp-30/
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

发布模型只用 30 条黄色胶棒示教训练。计划完整公开的数据集为
`xujiayuxian-png/xlerobot-glue-stick-grasp-30`，许可 CC BY 4.0。羽毛球效果只作为
定性的 OOD 证据。

## 资格检查与下载

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
