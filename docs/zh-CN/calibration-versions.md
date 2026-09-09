# 如何应用、替换和恢复标定配置

先区分两类文件：`config/local.yaml` 配置设备、服务和机器身份；
`.xlerobot/units/<unit>/` 保存这台实物机器的标定。模型权重不在这里。
`<unit>` 来自 `config/local.yaml` 的 `robot.unit_id`，并应与 `calibration.unit` 一致。
不要复制作者的设备零位或安装外参到另一台机器人，也不要直接覆盖运行 YAML。

## 目录与运行关系

```text
.xlerobot/units/<unit>/
├── draft/components/     分项新结果，尚未用于 Demo
├── versions/<version>/   不覆盖的完整版本
├── active -> versions/…  当前选择的版本
└── runtime/              从 active 生成、带校验和的运行配置
```

Demo 读取 `runtime/`，不直接读取 draft。`draft/runtime/<workflow>/` 只供后续标定采集，
不是 Demo runtime。手眼页面显示“保存到 draft”不等于已经替换 Demo。

## A. 新装机器人：首次应用

按[标定流程](calibration.md)完成自己的分项结果。`status` 应显示所需组件完整且通过校验：

```bash
./tools/calibrate status
./tools/calibrate activate --version first-calibration
./tools/calibrate render
./tools/calibrate status
```

确认 `active: first-calibration`、`runtime_matches_active: true`，再启动 Demo。
新机缺少参数时不能用下面的“同机保留旧值”跳过；它要求已经有完整有效的 active 版本。

## B. 同一台机器人：替换新测的部分

先停止 Demo、数采和会使用这些配置的运行程序，记下 `status` 中原 active 版本。
确认新草稿来自同一机器、相同的前序标定，安装变化后重采相关后续项。
下面一次换舵机、头相机和手眼结果，保留旧底盘、雷达及抓取补偿：

```bash
# 预览，不写入
./tools/calibrate replace --version calibration-v2 \
  --components servo head-camera right-handeye --dry-run
# 确认清单后替换：保存新版本、选择它并生成 runtime
./tools/calibrate replace --version calibration-v2 \
  --components servo head-camera right-handeye
./tools/calibrate status
```

预览中的 `replaced` 是本次替换项；`retained` 是沿用项，不代表重新测量通过。
`--components` 可以选择上述项目的子集，但不自动证明它们与其他旧组件兼容。
每次用一个新版本名；同名版本会拒绝覆盖。旧草稿、旧版本和采集数据不会因此删除。

## C. 已有版本：切换和恢复

```bash
./tools/calibrate switch --version calibration-v2
# 想恢复时：把 ORIGINAL_VERSION 换成之前记下的实际版本名
./tools/calibrate switch --version ORIGINAL_VERSION
./tools/calibrate status
```

`switch` 同时选择并生成运行文件，不需要再单独 `render`。
原有 `rollback --version …` 只切指针，还要 `render`，一般用户优先用 `switch`。
生成失败时工具尝试恢复先前版本；务必确认最后 `runtime_matches_active: true`。
切换只改本机文件，**不会驱动机器人，也不会热更新已经运行的节点**。
下一次启动 Demo/数采才会加载该版本。功能测试仍需在现场看护下进行。

## 哪些参数真正变化？

| 选择项 | 运行影响 |
| --- | --- |
| `servo` | `servos.yaml` 中左右从臂和头部的编码器零位、方向、范围；不包含 Leader |
| `head-camera` | `geometry.yaml` 中头相机安装平移/旋转 |
| `right-handeye` | `geometry.yaml` 中 Tag 23 相对固定夹爪的安装平移/旋转；完整求解保存于 `transforms.yaml` |
| 未选项 | 从当前 active 保留，包括底盘、雷达和抓取补偿 |

手眼求解的 `x` 是固定头姿态下的相机/FK 证据，当前不会直接覆盖相机 TF；
不会自动计算抓取补偿、修改 ACT 权重、或改写过去采集的数据。
Leader 是独立标定，见[Leader 入口](calibration.md#舵机网页整组活动一次完成)。

## 同机导入整套旧配置

只有同一台未改变相关安装的机器人，才适合从已知可用的完整快照导入：

```bash
./tools/calibrate import-runtime --input /path/to/snapshot --version adopted-runtime
```

快照必须含 `geometry.yaml`、`servos.yaml`、`controllers.yaml`、`transforms.yaml`、
`grasp_alignment.yaml`，并带来源信息；详见[后续指南](calibration-followup.md)。
该入口校验文件和数值完整性，不会把旧快照包装成新的实机精度验收。

## 排错

- `draft incomplete`：新机尚缺分项结果；同机部分替换用 `replace`，不要伪造缺项。
- `version already exists`：换新名称，或用 `switch` 选择已存在版本。
- `runtime checksum failed`：不要继续手改 runtime；先备份改动，再从有效版本 `switch` 重新生成。
- 切换后动作/TF 没变：运行节点没有重启，或误看了 `draft/runtime`；先检查 status，再重启正确工作区。
- 只想验证切换是否正常：对比前后文件/URDF，并切回原版本即可，不必驱动机器人。
