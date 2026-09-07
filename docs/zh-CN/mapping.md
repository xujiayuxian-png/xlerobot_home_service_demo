# 建图与保存地点

[文档目录](README.md) · 上一步：[标定](calibration.md) · 下一步：[Demo](demo.md)

地图描述**当前场地**，标定描述**当前机器人**，完整 Demo 两者都需要。
先完成标定激活与 render；启动建图前，停止其他 Demo 或数采工作区。

## 1. 建图

操作人员在机器人旁边时，在 Robot 主机执行：

```bash
./tools/run mapping --hardware --phase build
```

打开 `http://<robot-host>:8080`。如果传了 `--config PATH`，后面的验证和 Demo
也使用同一份配置。

- 点击 **开启遥控**，按住并拖动摇杆：上下前进/后退，左右转向，斜向可同时行驶和转向。
- 松手停止；离开窗口会结束遥控。速度滑条调整线速度和角速度上限。
- 遥控覆盖 Demo 将使用的区域，观察地图是否完整。

![建图工作区：离线界面预览](../images/mapping-ui.png)

*当前前端的离线布局截图，未连接机器人，不包含真实地图或相机画面。*

## 2. 记录桌边地点并保存地图

1. 结束遥控，等待机器人静止。
2. 将机器人停在面向桌子的最终贴桌位置，静止后记录地点 ID **`table`**。
   朝向和坐标同样重要；导航先到该位置后方 0.25 m，再执行精确贴桌。
3. 按需记录其他地点。选择已有 ID 可以更新或删除；更新使用机器人当前位姿。
4. 保存地图，检查页面显示的保存时间和地点列表。

保存得到的是**草稿**，还不是 Demo 正在使用的地图。保存后可以继续建图，
但新增扫描需要再次保存才能保留。

需要重建时，先结束遥控，再确认 **清除当前地图并重建**。
该操作仅重置当前 SLAM，不删除已保存地图、地点或 Demo 配置；
未保存的建图结果会丢失。重建后重新确认或记录地点坐标。

## 3. 验证并激活

Ctrl-C 停止 build，再启动独立的验证阶段：

```bash
./tools/run mapping --hardware --phase validate
```

1. 在页面执行定位，机器人可能旋转。
2. 逐个验证已保存地点的导航，机器人会移动。
3. 检查通过后激活草稿。

不要同时运行 build 和 validate。替换地图需要重新验证导航；
更新某个地点需要重新验证该地点。

## 4. 配置 Demo 使用的场地

激活文件位于
`<data.collection_root>/sites/<robot.site_id>/current/`。
使用示例配置时填写：

```yaml
site:
  map: .xlerobot/artifacts/sites/home-demo/current/map.yaml
  places: .xlerobot/artifacts/sites/home-demo/current/places.yaml
```

以工作区实际显示的路径为准，尤其是在改过保存根目录或场地 ID 时。
激活场地不会悄悄改写 Demo 配置。

完成标准：地图已保存，`table` 位姿正确，场地验证并激活，
配置中的两条路径指向这份结果。停止建图后，再启动 [Demo](demo.md)。
