# 两条抓取路线

ACT 是主 Demo 后端，centroid 和 GPD 只是可替换的抓取后端，不是另一套机器人栈。
三者共用激活的几何／舵机标定、带时间戳的 TF、抓取对齐和本地执行器工作空间；
传统感知不再额外要求头相机、手眼求解器的导出格式。
标定采样范围记录精度的测量来源，不是第二套运行工作空间。
复用本机旧标定不代表重新验证了精度，尤其不承诺接触高度的精度。

GPD 在 GPU 主机的 Ubuntu／WSL 中运行，与 SAM2 共用 8765 服务。
源码和构建放在本机状态目录 `.xlerobot/vendor/gpd`；干净安装使用
`./tools/setup gpu --with-gpd` 构建固定 revision。

两条路线共用同一套单机标定、物体请求、预抓取规划、本地 controller 路径和抓取
复核。选择通过 `ExecuteTask.grasp_backend` 显式传递，不在路线间静默 fallback。

## 传统 RGB-D 路线

8765 服务组合 VLM grounding、SAM 2 提示分割、D455 对齐深度、桌面/物体几何和
顶抓目标：

- `centroid` 是确定性的目标质心/顶抓几何，也是依赖最少的调试基线。
- `gpd` 使用固定 revision 的 GPD 产生候选，再经过同一套工作空间和顶抓筛选。

GPU 只返回感知候选，ROS manipulation 端拥有规划与执行。

## 混合 ACT 路线

`act` 先用标定后的视觉和 MoveIt 到达确定性预抓取，再向 8766 发送实测六关节状态
和 checkpoint 所需的腕部图像。返回的 100 步 action chunk 仍要经过 Robot 本地
streaming executor 校验。

模型合同、环境 pin 和 checkpoint 摘要见
`../../assets/models/act-local-grasp.md`。它只用 30 条黄色胶棒示教训练；羽毛球和
其他物体仅为定性 OOD 展示，不声明成功率或通用能力。权重采用 Apache-2.0，但首次
Hub 上传仍未完成，在 manifest 写入 immutable revision 前需使用已验证的本地权重。

## 对比运行

```bash
# GPU
./tools/run gpu

# Robot 终端 1
./tools/run demo --hardware

# Robot 终端 2；每轮只固定一种 backend
./tools/run grasp centroid --hardware
./tools/run grasp gpd --hardware
./tools/run grasp act --hardware
```

对比时固定物体摆放、active calibration、来源位置和照明，分别记录成功/失败与耗时。
