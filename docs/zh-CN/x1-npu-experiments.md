# X1 NPU 离线探索

日期：2026-09-15；基于 `x1-native` / `3f32737`。

本轮按任务目标选模型，不要求沿用原实现。先验证人员检测和中文短指令转写，
再试小语言模型意图解析。demo 保持停止；未打开电机、麦克风或相机，未发送任务。
实验只读取既有测试素材，调用本机 NPU，不调用远程 GPU。

## 1. 结论与接入边界

| 任务 | 本轮结果 | 后续选择 |
| --- | --- | --- |
| 人员检测 | YOLOv8n W8A8 在 QNN HTP 上输出人员框 | 最先考虑接入；只在搜索人员时运行 |
| 中文短指令转写 | Whisper-small / base W8A16 的编码器和解码器均在 HTP 上运行，三段音频各重复三次 | 优先试更省资源的 base；先移植拒识和超时恢复，再接任务入口 |
| 意图解析 | Qwen3-0.6B 能加载，QNN 2.36/2.40 均只生成结束符 | 未跑通；不替换现有解析，不把返回码成功当成任务成功 |
| 开放物品识别、抓取成功判断 | 本机 Qwen3-VL-4B 的一次服务加载试验段错误退出 | 未验证；不能据此认定模型本身不可用，也不能宣布端侧 VLM 已完成 |
| ACT 抓取 | 本轮未转换、未在 NPU 上推理 | 保留当前远程策略；需要实际权重导出、输入/动作一致性和实机验证 |

**这些是独立模型试验，不是完整 demo 的 CPU 验收。** 软件、NPU 推理正常不代表
现场能听清、能找人或能成功抓取。原有 CPU/远程配置保持不变。

## 2. 运行环境与证据

- X1 为 QCS8550 / Snapdragon，6 个在线 CPU，Ubuntu 22.04 aarch64。
- AidLite SDK / QNN236 / QNN240：`2.4.1.278`；AidGen：`2.3.0.131`。
- YOLO 显式选择 `TYPE_QNN236 + TYPE_DSP`；Whisper 显式选择 `TYPE_QNN240 + TYPE_DSP`。
- 报告记录实际加载的 `libQnnHtp.so`、V73 stub、SDK 插件，以及进程打开的
  `/dev/adsprpc-smd-secure`。没有使用 ONNX Runtime 的 CPU 回退来冒充 NPU。
- 模型、图片、音频和原始日志位于忽略的 `.xlerobot/npu/`、
  `.xlerobot/environment/asr-smoke/`，不提交模型权重或机器素材。

安装注意：尝试安装 AidVoice 时，包管理器实际依赖解析要求更新到 AidLite 2.5 /
QNN248，与查询页提示不同，因此中止。中止发生在同版本基础包重装阶段，随后
已重装恢复 `aidlux-aistack-base 1.3.1.169`；AidLite 保持 `2.4.1.278`，未安装
AidVoice。本轮直接使用 AidLite 完成 Whisper 推理，避免为一个接口升级整套环境。
`dpkg --audit` 中 libkmod2 / initramfs-tools 的遗留状态始于 9 月 12 日，未在本轮
顺带配置内核或重建 initramfs。后续 SDK 升级应独立安排并留回退副本。

## 3. 人员检测：统计方法

输入是 SDK 自带的同一张 `bus.jpg`，另外用全黑图检查明显误检。
QNN 模型固定 640×640；CPU 基线使用现有 `yolov8n.pt`、480 输入、2 个 PyTorch
线程。两者均置信阈值 0.6、只输出 person，每秒请求 3 次；按 NPU/CPU 交替顺序
各测三次 60 秒。预热和首次推理单独记录，不计入稳态。

端到端时间包括预处理、模型执行、输出读取和人员框后处理，不含磁盘图片读取、
ROS 图像传输、深度取点、TF 和机器人运动。CPU 占用按当前进程用户态及内核态
CPU 秒 / 墙钟秒统计：**100% 表示占满一个核，不是整机 100%**。

NPU 使用供应商的左上对齐补黑预处理，CPU 使用 Ultralytics 原有预处理。
输入尺寸、量化和预处理不同，这是一组候选实现的成本对照，不是等精度基准。
单张标准图片不足以证明室内遮挡、不同距离下的检测质量。

| 实现 | 三轮 CPU（单核 %） | CPU 均值 | 平均端到端时间 | 峰值进程 RSS |
| --- | --- | ---: | ---: | ---: |
| CPU / 480 | 117.587 / 118.671 / 117.989 | 118.082% | 174.34 ms | 408.2–408.4 MiB |
| QNN / 640 | 3.407 / 3.314 / 3.415 | 3.378% | 11.28 ms | 96.4 MiB |

进程 CPU 开销减少 **97.14%**；两版每轮均维持约 3 Hz、180 次推理，均检出
标准图中 3 个人，全黑图无检出。QNN 端到端 p95 为 11.87–12.69 ms，CPU 为
200.64–218.75 ms。原始报告为 `.xlerobot/npu/yolo-{qnn236,cpu}-{1,2,3}.json`。

当前 demo 已按动作需要取图，因此这些收益主要发生在人员搜索期间，不能直接从
53.36% 的未定位待机基线中扣除。进程 RSS 也未必包含全部 DSP/共享内存分配；
接入前仍需量整机可用内存、加载峰值与控制线程的周期。

## 4. 中文语音：已经跑通的部分

模型为 SDK 模型库的 Whisper-small / base W8A16，编码器输入固定 `[1,80,3000]`，
解码上下文 200。当前探针接收最多 30 秒、16 kHz 音频，固定中文、无时间戳，
贪心解码最多 64 步；没有麦克风监听、VAD、唤醒或意图分发功能。

三段既有测试音频每段三次，结果分别为：

- “帮我拿一下桌上的羽毛球。”
- “请把桌上的杯子拿过来。”
- “小乐小乐”

首次报告中耗时范围 0.31–0.65 秒，编码器约 60–87 ms，峰值进程 RSS 493 MiB。
这只证明短语转写；第三段能转写唤醒词，不等于已经有可靠的在线唤醒器。
当前浮点缓存拷贝仍占 CPU，不能把编码器的 60 ms 当成用户等待时间。

随后加入 CPU 基线、更小的 base 和静音测试。下表为每段音频三次的平均墙钟时间，
包含音频读取、特征计算、缓存搬运、解码及 NPU 置信度采集，不含模型加载：

| 测试音频 | CPU small / beam 3 | NPU small / greedy | NPU base / greedy |
| --- | ---: | ---: | ---: |
| 羽毛球，2.544 秒 | 4.478 s | 0.534 s | 0.228 s |
| 杯子，2.520 秒 | 4.259 s | 0.443 s | 0.189 s |
| 唤醒词，1.632 秒 | 4.157 s | 0.315 s | 0.144 s |
| 峰值进程 RSS | 1103.5 MiB | 492.7 MiB | 371.8 MiB |

羽毛球一段的进程 CPU 时间分别为 **7.900 / 0.483 / 0.217 CPU 秒**。
base 的三轮前两句为“幫我拿一下桌上的羽毛球。”和“請把桌上的杯子拿過來。”，
物品名和语义正确，但输出繁体；后续接入需要统一文本格式。两个 NPU 模型都是
三个不同样本，各重复三次，不能报告成九条不同指令的准确率。

**静音没有被原始解码器自动处理好。** 3 秒全零 PCM 的原始结果：CPU small
生成受提示词影响的文字（无语音概率约 0.817），NPU small 生成无关文字
（约 0.891），NPU base 反复生成无关文字直到 64 步上限（约 0.958）。
不能以非空输出或底层返回码成功作为识别成功。

探针增加离线候选筛选：必须解码正常结束、非空、无语音概率 ≤0.6、平均 token
对数概率 ≥−0.8，缺失/非有限证据拒绝。用采集结果检查，两个 NPU 模型的九次语音
均通过，三次静音均拒绝。此筛选没有向机器人分发任务；数值门槛沿用现有方案，
但贪心 token 概率与 faster-whisper 的统计口径并非完全相同，真实噪声、低音量、
电视声和口音仍需验证。不能把纯静音通过当成抗噪能力验证。

无语音概率读取 SOT 第一步 logits；本机 tokenizer 的名称为 `<|nocaptions|>`，
探针也兼容 `<|nospeech|>` 命名。原始对照分别保存在
`whisper-cpu-benchmark.json`、`whisper-{small,base}-confidence.json`，
后验筛选记录为 `asr-offline-screen.json`，均位于 `.xlerobot/npu/`。

实现中最关键的兼容细节：当前 AidLite 的非 native `set_input_tensor` 使用
FLOAT32 主机输入。即使模型 metadata 写着 INT32，token / position 仍需传
float32；传 int32 在本机不会报错，却会反复生成语言 token。探针以软件测试
固定此契约。未来更换 SDK 或 native 输入方式必须重新验证。

编码器的 cross attention cache 每条音频复制到解码器一次，self cache 每步
反馈，下一段重新清零；不复用上一段语音的状态。直接使用量化缓存可能减少
CPU 搬运，但必须先确认两个图的量化参数相同，不能直接复制原始字节。

## 5. 选择更省资源的方案

优先级按收益、任务频度和验证成本安排：

1. **人员检测先接入小检测器。** 不为找人启动 VLM。保留现有图像新鲜度、
   深度定位、取消释放和控制端安全检查；检测模型只返回框和置信度。
2. **唤醒维持轻量关键词检测，转写按需运行。** 本轮证明 small 模型可运行，
   更小的 base 已取得更低成本，因此优先作为接入候选。模型库还列出 tiny 和
   SenseVoiceSmall，本轮没有运行它们；只要通过开放物品名、噪声和漏字检查均可
   替换。固定提示继续播放预制 WAV，不引入 TTS。后续也应减少 NPU 探针对
   faster-whisper 的特征/音频工具依赖，避免仅为预处理加载完整 CPU 推理库。
3. **意图解析不强制生成式大模型。** 高频明确句式可用轻量槽位解析，物品名
   从原句提取而非限制为类别表；歧义、否定、复合指令需要可靠拒识或现有解析
   兜底。应与小分类/序列标注模型、小 LLM 比较完整命令正确率和 CPU 秒。
4. **物品识别与抓取验证拆开评估。** 人物定位、目标区域匹配、前后画面变化
   不必全部用一个 VLM。开放名称到图像区域的对应仍是硬要求；固定 COCO 类别
   检测器无法替代。可评估开放词汇检测/图文匹配与按需 VLM，但本轮没有实测
   这些候选，不能提前承诺效果或负载。
5. **ACT 独立迁移。** 保留实际训练策略和动作含义，先导出并对同一离线序列
   比较动作，再评估量化。更小的通用模型不能凭空替代已训练的抓取技能。

NPU 模型不需要全部常驻。ASR、目标识别、ACT、递送找人通常处于不同任务阶段，
可考虑分阶段加载或共享执行资源；先测加载时延和内存再选择常驻组合。并发争用、
长期温升、崩溃恢复仍未测试，暂不将多个新后端同时接入 demo。

## 6. 复现与软件验证

公开入口沿用 `tools/doctor`，不需要 `--hardware`；命令不会访问电机设备：

```bash
tools/doctor robot --npu-bench yolo --help
tools/doctor robot --npu-bench whisper --help

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 tools/doctor robot --npu-bench yolo \
  --backend qnn236 --duration 60 --rate 3 \
  --model .xlerobot/npu/models/yolov8n/models/QCS8550/W8A8/cutoff_yolov8n_qcs8550_w8a8.qnn236.ctx.bin \
  --image .xlerobot/npu/models/yolov8n/code/python/bus.jpg \
  --output .xlerobot/npu/yolo-repeat.json

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 tools/doctor robot --npu-bench whisper \
  --backend qnn240 --repeat 3 \
  --model .xlerobot/npu/models/whisper-small/models/QCS8550/W8A16 \
  --audio .xlerobot/environment/asr-smoke/shuttlecock.wav \
          .xlerobot/environment/asr-smoke/cup.wav \
          .xlerobot/environment/asr-smoke/wake.wav \
  --output .xlerobot/npu/whisper-repeat.json
```

CPU 对照使用同一入口：YOLO 改为 `--backend cpu --size 480 --model
.xlerobot/models/yolov8n.pt`；ASR 改为 `--backend cpu --model
.xlerobot/models/faster-whisper-small`。CPU ASR 固定当前中文 beam=3、int8、2 线程，
沿用取物提示词；NPU 为无提示词贪心，两者不是完全相同的解码搜索。

模型来自已安装的 `mms` 工具：YOLOv8n / int8 / qcs8550 / qnn2.36，
Whisper-small / w8a16 / qcs8550 / qnn2.40。下载及解包只放忽略目录。
base 对照将模型选择改为 `Whisper-base`，输出目录改为 `whisper-base`；相同探针
直接兼容已验证的这两个 W8A16 artifact，不承诺支持任意同名导出模型。
`tools/lib/npu_text_probe.cpp` 是排查 AidGen 的内部离线程序，需本机 SDK 开发头文件，
不是生产意图后端；目前返回码成功并不代表输出是有效 JSON。
该模型的字符串生成路径仅返回结束符；尝试 `--embedding` 路径时 SDK 明确报
`Failed to create the extractor`。尚未定位是配置、模型包还是运行时兼容问题。
内部探针的编译方式（从 `ros2_ws` 执行）：

```bash
g++ -std=c++17 -O2 -pthread -I/usr/local/include \
  ../tools/lib/npu_text_probe.cpp -L/usr/local/lib -laidgen \
  -o ../.xlerobot/npu/npu_text_probe
```

软件测试只使用合成数组和假 SDK，覆盖人员框过滤/NMS/裁剪、解码掩码及输入类型、
模型加载失败的资源释放。没有将 NPU 性能测试加入日常 CI，也不依赖机器权重。
还覆盖概率计算数值稳定性、静音/未完成解码/缺失证据的拒绝。`xlerobot_bringup`
从 `ros2_ws` 构建成功，新测试 5 项全部通过；AidGen 探针从同一目录用本机 C++ SDK 编译。

参考当前本机 SDK 头文件及供应商资料：
[AidLite](https://docs.aidlux.com/en/software/ai-sdk/aidlite_guide)、
[AidVoice](https://docs.aidlux.com/en/software/aidvoice/aidvoice_guide)、
[AidGen](https://docs.aidlux.com/en/software/genai-sdk/aidgen/aidgen_guide)。
Whisper 解码掩码和缓存反馈参考
[Qualcomm HF Whisper 应用](https://github.com/qualcomm/ai-hub-models/blob/main/src/qai_hub_models/models/templates/hf_whisper/app.py)；
本机 artifact 的形状、接口 dtype 和运行结果以实测为准。
