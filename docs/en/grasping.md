# Two grasp routes

[Documentation](README.md) · [中文](../zh-CN/grasping.md)

ACT is the main demo route. Centroid and GPD are alternative grasp backends,
not separate robot stacks. All share the active robot calibration, task
interfaces, pregrasp planning and local controller path.

| Backend | Target/contact method | GPU dependency |
| --- | --- | --- |
| `act` | RGB-D coarse target → MoveIt pregrasp → wrist-image ACT chunks | LM Studio + ACT on 8766 |
| `centroid` | VLM box → SAM 2 → RGB-D body/table geometry → top grasp | LM Studio + SAM 2 on 8765 |
| `gpd` | Same segmented cloud → GPD candidate ranking → constrained top grasp | LM Studio + SAM 2/GPD on 8765 |

## Hybrid ACT

ACT handles the local contact phase, not navigation or the whole arm approach.
The GPU receives measured six-joint state and wrist RGB, returning 100-step
action chunks. The robot-local streaming executor validates ordering and
command bounds before execution.

The [model card](../../assets/models/act-local-grasp.md) records structure and
limits. Training used 30 yellow-glue-stick demonstrations only.
Shuttlecock and other objects are qualitative generalization examples.

## Classical geometry

Centroid and GPD share segmentation, calibrated frames and workspace filtering.
The manipulation side plans the descend/close/lift sequence with MoveIt.
The GPU returns candidates only; it never controls motors.

This implementation supports the reference **top-grasp** geometry, not general
6-DoF grasping. A GPD failure stays a GPD failure, without silent centroid fallback.
Install GPD in GPU Ubuntu/WSL with `./tools/setup gpu --with-gpd`; the pinned
native build is under ignored `.xlerobot/vendor/gpd`.

## Run one backend per trial

Complete [demo prerequisites](demo.md), then start once:

```bash
# GPU: start LM Studio first
./tools/run gpu

# Robot, terminal 1
./tools/run demo --hardware
```

From a second Robot terminal, execute **one** of:

```bash
./tools/run grasp act --hardware
./tools/run grasp centroid --hardware
./tools/run grasp gpd --hardware
```

These commands submit a **full fetch-and-deliver task**, not just an arm action.
For yellow-glue-stick trials, set `demo.object_id: 黄色胶棒` before submitting
the CLI request. The web UI also accepts an object name and explicit backend;
voice uses the configured default backend.

Use consistent object placement, active calibration, table place and lighting
for comparisons. Record the requested and actual backend and the final result.
Do not interpret a single successful trial as a statistical success rate.
