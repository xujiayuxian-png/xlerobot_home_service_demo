---
license: cc-by-4.0
task_categories:
  - robotics
tags:
  - lerobot
  - xlerobot
  - imitation-learning
language:
  - zh
---

# XLeRobot yellow-glue-stick grasp: 30 demonstrations

All **30 original demonstrations** used to train
`xujiayuxian-png/xlerobot-act-local-grasp-v1`. They show the reference right arm
grasping a yellow glue stick, with the task text “拿起黄色胶棒”.
This is not the later collection-tool acceptance dataset.

## Contents

- LeRobot v3.0: **30 episodes, 6,149 training frames, 30 Hz**.
- `data/`: Parquet state/action and per-frame indices.
- `meta/`: episode ranges, task table, feature definitions and statistics.
- `videos/`: head and wrist RGB streams.
- The ACT checkpoint uses the wrist stream only; head video remains available
  as observation context.

State/action joint order: right shoulder pan, shoulder lift, elbow flex,
wrist flex, wrist roll, gripper. In the original recordings, the raw action
array duplicated state. The original training conversion instead constructed
`action[t] = state[t+1]` and dropped each episode's final frame.
The published converted data is the training input; do not train on the
uncorrected raw action array.

These recordings predate the new collection tool's `raw/` + `reviews/` format.
They do not gain new-tool review sidecars or unit-calibration manifests by being
published. No such provenance is fabricated.

## Use

Download the full converted dataset, retaining its directory layout, then pass
that root to the source repository's `tools/act train --dataset PATH`.
Do not run `tools/act convert` on it: that command is for new raw collections.

Dataset size is about 235 MiB before ancillary publication metadata. The dataset
contains only glue-stick demonstrations, not shuttlecock demonstrations.
Reproducing a usable policy still depends on matching hardware, observation
conventions, calibration and training configuration.

## Limits and attribution

This is a small, single-reference-robot dataset, not a benchmark. It does not
establish success rates, broad object coverage or general grasp capability.

Licensed **CC BY 4.0**. Attribute “XLeRobot Home Service Demo contributors” and
this dataset, link the license and indicate changes when redistributing adaptations.
License text: https://creativecommons.org/licenses/by/4.0/legalcode

## Availability

Intended dataset ID: `xujiayuxian-png/xlerobot-glue-stick-grasp-30`.
Initial upload is pending. The source repository's data manifest records the
immutable publication revision once available. Large data and videos are not
committed to the source Git repository.
