# X1 4B 识别退化定位

2026-09-16。机器人 demo 始终停止；本次仅使用历史头部图像、离线 SDK
和一次 GPU 对照，不增加生产 GPU 依赖。

## 结论

问题已经收敛到当前 Qwen3-VL-4B NPU 转换包及其多模态执行链路。
不能把这次表现解释为“4B 原模型不如 3B”，也不能用升级其他服务的
运行库来解决。现有证据不足以把所有错误都归因于单一因素。

确认的结构差异是：当前路径没有传递官方模型需要的 DeepStack 多层视觉特征。
这是重大兼容性问题，不是只改输入分辨率、提示词或坐标缩放就能补齐的。
尚未完成 DeepStack 消融对照，不能宣称其为全部误差的唯一原因；
文本侧 MRoPE 与量化误差也仍待验证。

## 同图对照

所有下表请求都使用同一张 640×480 头部原图，双三次缩放至 448×448，
相同中文提示词，明确要求 0–1000 归一化坐标。
GPU 使用原有 LM Studio Qwen3-VL-4B GGUF 服务，仅用于诊断。
GPU 与 NPU 的量化格式不同，所以这不是单变量量化精度实验。

| 路径 | 羽毛球输出框 | 蓝色打火机输出框 |
|---|---|---|
| NPU：HTTP、AidLite 2.5 | `[477,727,537,807]` | `[677,577,737,630]` |
| NPU：绕过 HTTP，直接 AidGen + 厂商视觉编码示例、AidLite 2.4 | `[477,727,537,807]` | `[677,577,737,630]` |
| NPU：直接 SDK，恢复厂商默认生成线程数 | `[477,727,537,807]` | `[677,577,737,630]` |
| GPU：相同 448 输入、相同提示 | `[485,587,577,700]` | `[612,595,657,687]` |

GPU 框覆盖可见目标；NPU 羽毛球框落在目标下方，打火机框偏到右边。
这两项上，直接 SDK 与 HTTP 的错误结果完全相同，排除了 demo
坐标解析、HTTP 封装、新旧图像运行库和低待机线程配置作为差异来源。
相同分辨率下 GPU 正常，说明 448 分辨率不足不是这些偏框的充分解释。
空白图对照返回 `found=false`，表明图像确实影响输出，并非完全未送入图像。

## 结构证据

[官方配置](https://huggingface.co/Qwen/Qwen3-VL-4B-Instruct/raw/main/config.json)
定义 `deepstack_visual_indexes=[5,11,17]`。
[官方实现](https://github.com/huggingface/transformers/blob/v4.57.1/src/transformers/models/qwen3_vl/modeling_qwen3_vl.py)
在视觉编码器返回主图像特征之外，还返回三个中间层特征，并注入语言模型前几层。

本机检查发现：

- 正在使用的完整 AIDEM 视觉模型仅有一个输出 `image_features=[196,2560]`。
- 对应 raw 包视觉模型也仅有该输出。
- raw 包三个 LLM 分片的全部图输入已枚举，除了 embedding / 中间状态、KV cache、位置编码和 attention mask，没有额外视觉特征输入。
- 厂商 SDK 示例只拼接文本 embedding、这一份视觉 embedding、文本 embedding；没有 DeepStack 注入。
- raw 包与完整 AIDEM 包在本次两项同图请求上输出相同。

完整 AIDEM 的 LLM 部分采用 AidGen 专用封装，不能用 AidLite 直接读取；
该尝试返回不支持的模型类型。因此“LLM 图输入枚举”证据明确来自对应 raw 包，
不能描述成已解读完整 AIDEM 的全部内部节点。

图像预处理也做了核对：官方 `patch_size=16`、`temporal_patch_size=2`、
`merge_size=2`、均值/标准差均为 0.5；厂商示例的参数、归一化及 patch
排列与这些要求一致。[预处理配置](https://huggingface.co/Qwen/Qwen3-VL-4B-Instruct/raw/main/preprocessor_config.json)
这只是已检查部分，不等于证明整个转换图与官方逐层数值一致。

## 没有采用的处理

没有人为平移检测框，没有根据理由文字强行改写抓取成功标志，
没有悄悄改用 GPU，也没有据此更换其他参数规模的模型。
本次没有修改生产识别代码或 `local.yaml`；4B 配置保留，demo 停止。

## 正确修复方向

1. 取得或重新导出保留三路 DeepStack 特征及注入点的 4B NPU 模型；
   对照官方实现核对 MRoPE。仅补 Python 参数无法修复已经裁掉的图输入。
2. 在接入 demo 前，用本次两张目标对照和腕部抓取样本验证其行为；
   不再用“成功加载、返回 HTTP 200”作为可用标准。
3. 若短期仍用现有包，只能视作存在已知质量缺陷的实验后端；
   不将它宣称为完成了对 3B 的效果升级。

## 证据位置

后续已跑通 GenieX + 完整 GGUF 的独立 X1 实验，见
[官方运行时实验记录](x1-qwen3-vl-4b-official-experiment.md)。
保留原图尺寸后，羽毛球和蓝色打火机的框均明显改善；尚未接入 demo。

本机忽略目录 `.xlerobot/npu/qwen3-audit/`：

- `native.cpp`：直接调用 SDK 的离线驱动，复用机器安装的厂商图像编码示例。
- `results-native.json`：直接 SDK、两个物品及空白图。
- `results-default.json`：默认生成线程对照。
- `gpu-448.json`：同分辨率 GPU 原始响应。
- `tensors.json`：raw 包全部图输入输出。
- `tensors-aidem.json`：完整包视觉模型输入输出。

直接 SDK 的开放式场景描述试验未在 90 秒进程预算内完成，超时终止，
没有把它记成有效成功样本。最初将新版库与旧版头文件混编导致的示例崩溃
也已单独定位为 ABI 问题；最终对照使用匹配的头文件与运行库。
