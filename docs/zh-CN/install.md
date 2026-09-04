# 源码安装

## 支持环境

Robot 与 GPU 是两台 Ubuntu 24.04 x86_64 电脑。Robot 安装 ROS 2 Jazzy；已验证
GPU 环境是 WSL2 Ubuntu 24.04 + RTX 3080。其他系统、ROS 版本和 GPU 栈不属于
首版支持范围。

两台电脑均执行：

```bash
git clone --recurse-submodules <repository-url>
cd xlerobot_home_service_demo
cp config/local.example.yaml config/local.yaml
cp .env.example .env
```

编辑本地文件。两端使用相同、重新生成的 `XLEROBOT_ACT_TOKEN`，不要把 8765/8766
暴露到公网。

## GPU 端

先安装 Miniconda，再运行：

```bash
./tools/setup gpu
```

这会准备默认的 ACT 和传统 centroid 路线。如需编译可选的、固定版本的 GPD 原生
候选生成器及系统依赖，请改用 `./tools/setup gpu --with-gpd`。

脚本建立两个互不混装的源码环境：

- `.xlerobot/venvs/classical`：Python 3.10.20 与传统路线依赖。
- `.xlerobot/venvs/act`：Python 3.12.13 与 ACT 依赖。

setup 会显式下载并校验固定 revision 的 SAM 2。另行启动 LM Studio `0.4.12+1`，
按 manifest 中的 revision/hash 加载 Q4_K_M 工件，并以 `qwen/qwen3-vl-4b`
对外服务。doctor 可核对 API 名称，本地 GGUF 身份仍需在 LM Studio 内对照清单。
先阅读 `../../assets/models/manifest.yaml`，把已验证 ACT checkpoint 的
repo-relative 路径填到 `models.act_checkpoint`，并把对应 qualification 记录放到
`models.act_manifest`。首次 Hub 发布前，两者
必须从已验收的本地训练结果一起复制（生成方法见 `act-workflow.md`）；写入 immutable
Hub revision 后，`tools/act download` 才能下载并逐文件校验两者。

## Robot 端

以下命令运行 rosdep、建立 `.venv/robot`、构建 Web 前端并从源码构建 ROS：

```bash
./tools/setup robot --with-person-detector --with-kws-model --generate-voice-prompts
```

三个附加参数因许可证或来源边界而显式启用：person detector 安装 AGPL-3.0 Ultralytics，
从清单中的 provider URL 下载 `yolov8n.pt` 并核对大小与 SHA-256；voice prompts
调用 Edge TTS，在本机生成被 Git 忽略的 MP3。`--with-kws-model` 会先提示上游
模型/词表条款仍不明确，再从原提供者直接下载；语音模式需要它，本仓不镜像该模型。
历史 MP3 不随仓发布。

setup 默认下载 MIT 许可且由 manifest 固定的 Whisper snapshot，并逐文件校验；
只有显式传入 `--with-kws-model` 才下载和校验 KWS。这与
`--generate-voice-prompts` 不同，后者只负责生成本机播报 MP3。

机器路径、设备别名、LAN 地址和 Demo 默认值只写入 `config/local.yaml`；密钥只写
`.env`。地图、具名位置、标定、模型和录制数据放在仓库内被忽略的 `.xlerobot/`
目录，不进入 Git。
