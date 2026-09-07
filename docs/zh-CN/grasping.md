# 两条抓取路线

[文档目录](README.md) · [English](../en/grasping.md)

ACT 是主 Demo 路线；centroid/GPD 是可替换后端，不是另一套机器人运行栈。
三者共用激活标定、任务接口、预抓取规划和本地 controller 路径。

| 后端 | 目标或接触阶段的实现 | GPU 依赖 |
| --- | --- | --- |
| `act` | RGB-D 粗目标 → MoveIt 预抓取 → 腕部图像 ACT chunks | LM Studio + 8766 ACT |
| `centroid` | VLM 框 → SAM 2 → RGB-D 物体/桌面精炼 → 质心顶抓 | LM Studio + 8765 SAM 2 |
| `gpd` | 同一分割点云 → GPD 候选排序 → 约束顶抓 | LM Studio + 8765 SAM 2/GPD |

## 混合 ACT

ACT 负责局部接触阶段，不负责导航或整段机械臂接近。
GPU 接收实测六关节状态和腕部 RGB，返回 100 步 action chunks；
机器人本地 streaming executor 核对顺序和命令范围后执行。

结构和边界见[模型卡](../../assets/models/act-local-grasp.md)。
权重只用 30 条黄色胶棒示教训练；羽毛球及其他物品是定性泛化展示。

## 传统几何

Centroid/GPD 共用分割结果、标定坐标系和工作空间筛选。
Manipulation 端用 MoveIt 规划下降、闭爪和抬升；GPU 只返回候选，不控制电机。

当前只支持参考配置的**顶抓**，不是通用 6-DoF 抓取。
GPD 失败就报告失败，不静默切换为 centroid。
在 GPU Ubuntu/WSL 使用 `./tools/setup gpu --with-gpd` 安装，
固定版本的原生构建位于被忽略的 `.xlerobot/vendor/gpd`。

## 每轮选一个后端

完成 [Demo 前置步骤](demo.md)，先启动一次：

```bash
# GPU：先启动 LM Studio
./tools/run gpu

# Robot 终端 1
./tools/run demo --hardware
```

在第二个 Robot 终端，每轮只执行其中一条：

```bash
./tools/run grasp act --hardware
./tools/run grasp centroid --hardware
./tools/run grasp gpd --hardware
```

这些命令提交的是**完整取物递送任务**，不只是机械臂动作。
测试黄色胶棒时，先设置 `demo.object_id: 黄色胶棒` 再提交命令行请求。
网页也可直接填写物品并选择后端；语音使用配置中的默认后端。

对比时固定摆放、active 标定、桌边地点和照明，记录请求/实际后端及最终结果。
单次抓取完成不等于统计成功率。
