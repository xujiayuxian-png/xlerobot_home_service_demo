# 模型与数据

[文档目录](README.md) · [English](../en/assets.md)

## 直接使用 Demo，无需训练

ACT 权重与源码分开分发。推理不需要下载 30 条训练数据。

| 资产 | 来源或取得方式 |
| --- | --- |
| ACT 权重 | `lissajous/xlerobot-act-local-grasp-v1`；已公开 |
| 30 条黄色胶棒示教 | `lissajous/xlerobot-glue-stick-grasp-30`；已公开，推理不需要 |
| SAM 2 | `tools/setup gpu` 下载并校验 |
| Qwen VLM | 在 LM Studio 加载指定模型，见[安装说明](install.md) |
| Whisper | `tools/setup robot` 下载并校验 |
| KWS | 显式 `--with-kws-model` 从提供方下载，模型条款尚未明确 |
| Person detector | 显式 `--with-person-detector` 安装 AGPL 附加项 |
| 语音反馈音频 | `--generate-voice-prompts` 本地生成 |

精确版本和 SHA-256 记录见[模型清单](../../assets/models/manifest.yaml)、
[ACT 下载 manifest](../../assets/models/xlerobot-act-local-grasp-v1.manifest.json)、
[语音 manifest](../../assets/models/voice-runtime.manifest.json)、
[数据清单](../../assets/data/manifest.yaml)。两个项目资产仓库均已公开，下载无需 Hub 登录。
[数据 SHA256 清单](../../assets/data/xlerobot-glue-stick-grasp-30.sha256.json)
记录了实际转换训练文件的身份。

在 GPU 主机完成 setup 后执行：

```bash
./tools/act download
./tools/doctor gpu
```

下载入口将权重和 manifest 安装到配置中的 `models.act_checkpoint` /
`models.act_manifest` 路径，并检查摘要。Doctor 仍可能提示尚未运行的 LM Studio
或推理服务，接着按 [Demo 启动说明](demo.md)操作。

需要替换模型时，[自行训练并检查](act-workflow.md)。
不要把短步数软件验收模型当成展示权重。

## 训练数据

GPU setup 完成后，下载固定公开版本的已转换数据：

```bash
.xlerobot/venvs/act/bin/hf download lissajous/xlerobot-glue-stick-grasp-30 --repo-type dataset --revision 31e398471db6815d7ea900c0954f4e541d343f6e --local-dir .xlerobot/datasets/xlerobot-glue-stick-grasp-30
```

仓库附带 `checksums.json`，记录数据、元信息和视频文件摘要。
也可以直接查看 Hub 上的[数据卡](https://huggingface.co/datasets/lissajous/xlerobot-glue-stick-grasp-30)
和[模型卡](https://huggingface.co/lissajous/xlerobot-act-local-grasp-v1)。

发布的是原始 30 条黄色胶棒示教，不是后续工具验收时采集的数据。
见[数据卡](../../assets/data/glue-stick-grasp-30.md)。发布的 LeRobot 数据集是训练输入，
不等于自动拥有 `raw/` 和 `reviews/` 的原始数采工作区。

自己采集的数据走[数采 → 转换 → 训练](act-workflow.md)；
已经转换好的 LeRobot 数据集直接传给 `tools/act train --dataset PATH`，
不要再执行一次原始数据转换。

## 条款与能力边界

本项目 ACT 权重采用 Apache-2.0，示教数据采用 CC BY 4.0。
其他模型使用提供方条款，见[第三方声明](../../THIRD_PARTY_NOTICES.md)。
权重只用黄色胶棒示教训练；其他物品仅为定性演示，不提供成功率统计。
