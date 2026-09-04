# 完整取物递送 Demo

操作流程和 Web UI 可先看
[HMI 演示视频](https://www.bilibili.com/video/BV1GSK66XEqf)。

```text
语音/Web/CLI 请求 → 定位 → 导航并靠桌
→ 物体 grounding → 指定抓取路线并复核
→ 寻找最近人员 → 接近 → 语音反馈 → 递送（随后回到 ready）
```

启动前应保证：稳定设备别名存在；完整标定已 active/render；Nav2 地图和具名位置与
当前场地一致并包含 `table`；Robot 能访问 LM Studio、8765 和 8766；寻人模式的
person model 可读；语音开启时本地语音模型与音频设备可用。

先运行 `tools/doctor robot` 并解决全部 `ERROR`。

```bash
# GPU
./tools/run gpu

# Robot
./tools/run demo --hardware
```

Web 默认地址为 `http://<robot-host>:8080`。默认语音/Web 任务是 ACT 抓取
`羽毛球`；这是相对于 30 条黄色胶棒训练数据的定性 OOD 展示。也可以在第二个
Robot 终端固定路线：

```bash
./tools/run grasp act --hardware
```

更换标定、接线、地图或 controller 配置前，先 Ctrl-C 停止前台 Robot 进程。
Ctrl-C 停止 `tools/run gpu` 时会同时结束两个推理服务。
