# Documentation

[中文](../zh-CN/README.md) · [Project overview](../../README.md)

## Run the existing demo

Use this path to reproduce the video with the published ACT checkpoint.
You do **not** need a Leader, a dataset download or a training run.

1. [Hardware](hardware.md): reference build, wiring and device names.
2. [Install and configure](install.md): Robot/GPU environments and local settings.
3. [Models](assets.md): obtain the inference assets.
4. [Calibrate](calibration.md): measure, validate and manage your unit in the [unified web workspace](calibration-workbench.md).
5. [Map the site](mapping.md): save the map and `table`, validate, activate.
6. [Run the demo](demo.md): start services, check readiness, use voice or web.
7. [Compare grasp backends](grasping.md): ACT, centroid and GPD in the same task.

## Collect and train your own grasp

Once the robot is calibrated, connect the right Leader and follow
[ACT collection, conversion and training](act-workflow.md). This workflow does
not require navigating around a mapped site.

For either path, see [troubleshooting](troubleshooting.md).
Select **EN** in the top bar to match the English screenshots and button names
in these guides. The Demo and tool pages have an **EN / 中文** switch and the
browser remembers the selection. Screenshots are labelled offline layout
previews, not robot test evidence.

[After calibration](calibration-followup.md): base/lidar measurements, optional grasp
alignment, tabletop hover checking, and calibration version replacement.

[Calibration replacement manual](calibration-versions.md): first installation,
selective replacement, switching and restoration.

## Source layout

| Location | What to look for |
| --- | --- |
| `tools/` | Five public command entry points |
| `config/local.example.yaml` | Machine configuration template |
| `ros2_ws/src/` | ROS packages: shared task, robot drivers, perception, manipulation, HMI and tools |
| `services/classical/` | GPU SAM 2 / GPD proposal service |
| `services/act/` | ACT HTTP inference and training workflow |
| `examples/calibration/` | Small solver inputs and expected replay results |
| `examples/grasping/` | Shared geometry fixture |
| `assets/` | Printable targets, model/data metadata; no large weights |
| `.xlerobot/` (ignored) | Your machine's maps, calibration, recordings, models and logs |

Only contributors need to inspect the internal ROS package boundaries.
