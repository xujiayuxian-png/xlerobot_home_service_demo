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

接好 `robot.devices.leader_arm` 指定的单个右臂 Leader，并渲染 active 标定后，在
Robot 端运行：

```bash
./tools/act collect --hardware
```

它会启动源码采集工作区，右侧 Follower 机械臂可能运动。通过 Web UI 审核每条不可变
episode；原始录制不进入 Git。

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
