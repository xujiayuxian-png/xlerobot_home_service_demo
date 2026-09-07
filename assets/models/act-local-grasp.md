---
license: apache-2.0
library_name: lerobot
tags:
  - robotics
  - act
  - xlerobot
  - imitation-learning
datasets:
  - xujiayuxian-png/xlerobot-glue-stick-grasp-30
language:
  - zh
---

# XLeRobot ACT local grasp v1

ACT for the **local contact phase** of a two-wheel XLeRobot's right-arm grasp.
The robot first uses calibrated RGB-D perception and MoveIt to reach pregrasp.
This model then proposes action chunks from the wrist image and measured arm
state. Commands are executed by the robot-local streaming executor, not the GPU.

[Demo video](https://www.bilibili.com/video/BV18RK66JEdP)

## Data and limits

Trained only on **30 yellow-glue-stick demonstrations** (6,149 converted
training frames at 30 Hz). The corresponding dataset is
`xujiayuxian-png/xlerobot-glue-stick-grasp-30`, licensed CC BY 4.0.
Shuttlecock and other-object examples are qualitative generalization
demonstrations. No statistical success rate or general grasp capability is claimed.

The released checkpoint is the demonstrated inference checkpoint, not the
short-training artifact used to test the collection/conversion pipeline.
Calibration, reference hardware and the pregrasp stage remain necessary.

## Input and output

- Input: `observation.images.wrist`, RGB `3 × 480 × 640`, and
  `observation.state`, six measured joint values.
- Output: 100 × 6 joint-position actions.
- Joint order: shoulder pan, shoulder lift, elbow flex, wrist flex, wrist roll,
  gripper, corresponding to the reference robot's right arm.
- Training action semantics: next measured Follower state; the terminal frame
  without a next-state target is excluded during original conversion.
- Both image streams are present in the dataset, but this checkpoint uses
  **only the wrist image**.

## Architecture and runtime

ACT with VAE, ResNet-18 (ImageNet V1 initialization), dimension 512, feed-forward
dimension 3200, four encoder layers, one decoder layer, eight heads and latent
dimension 32. Chunk size and action steps are 100; AMP is disabled.

Verified runtime: Ubuntu 24.04 WSL2, RTX 3080, Python 3.12.13,
LeRobot 0.5.1, PyTorch 2.10.0+cu128 and torchvision 0.25.0+cu128.
The source repository pins the remaining dependencies.

## Files and use

Keep `config.json`, `model.safetensors`, both processor JSON files and both
normalization safetensors files together. `train_config.json` records the
original training configuration; machine-local paths in that record are
provenance, not paths that another user should create.
No optimizer state is included: this is not a full resumable training checkpoint.

Use the source repository's `tools/act download` after the immutable public
revision is recorded, then configure the GPU service and run through the robot
stack. `tools/act evaluate` checks deployment compatibility and hashes; it is
not a physical grasp benchmark.

## Availability and license

Apache-2.0. Intended model ID:
`xujiayuxian-png/xlerobot-act-local-grasp-v1`.
The initial upload is pending; the source repository's download manifest
remains the source of truth for availability, revision and exact file hashes.
