# ACT: collect, convert and train

[Documentation](README.md) · [中文](../zh-CN/act-workflow.md)

This is optional for running the demo: use the [published checkpoint](assets.md)
unless you want to train your own grasp. The [ACT video](https://www.bilibili.com/video/BV18RK66JEdP)
shows the calibrated pregrasp followed by learned contact.

## 1. Start collection on Robot

Prepare the calibrated robot, D455, wrist camera, object and the single right
Leader configured at `robot.devices.leader_arm`. Stop the demo or mapping
workspace first. Collection does not require a site map.

```bash
./tools/act collect --hardware
```

Open `http://<robot-host>:8080` (or the configured `demo.web_port`).
Startup moves only the head to centered pan and **0.8 rad tilt** over 3 seconds,
not either arm or gripper. Set another calibrated viewing angle at launch with
`--head-tilt RAD`; keep it fixed throughout a dataset.

![Collection workspace, offline layout preview](../images/collection-ui.png)

*Current frontend with example identifiers; no robot connection or camera data.*

## 2. Record one episode

| Action | What happens / what to wait for |
| --- | --- |
| Choose dataset, object and template | Changing the object auto-fills the instruction; you can override it. |
| **开始 / 准备 pregrasp** (Start) | Pick mode detects the object, opens the Follower gripper and prepares both arms concurrently. Wait for `WAITING_HOME`; Leader torque remains on, no recording yet. |
| **Home / 释放主臂并开始采集** | Support the Leader, then press Home. Wait for `RECORDING` before moving it; following and recording start and Leader torque is released. |
| **End / 结束并保存到本机** | Stops recording and saves locally. Following continues so you can put down the object and return. Those later movements are not recorded. |
| Next **Start** | Stops between-episode following, then prepares a new episode. No existing episode is overwritten. |

**抓取模板** (pick) uses the demo's shared perception/pregrasp path and needs its
VLM connection. **通用手动** (manual) skips perception and automatic Follower
pregrasp, aligning the Leader to the current Follower pose. Manual mode with
a 10–15 second limit is useful for a first recording.

Home/End also work as keyboard shortcuts, except while typing or holding a key.
Home first obtains the initial sample and checks alignment while holding
torque. Wait for `RECORDING`, not merely the button click.
Waiting for Home beyond 60 seconds cancels the attempt; it does not auto-start.
Cancellation/preparation failure can release Leader torque, so keep it supported.

Reaching the duration limit acts like End. **Abort** instead cancels the episode
and retains incomplete data; it is not the normal save button.
The displayed frame count is the actual recorder count.
The UI's state-machine dry-run does not record data or prove hardware readiness.

## 3. Keep or reject

A successfully saved, complete recording is **kept by default**. Do nothing to
use it for training; click **拒绝本条** to exclude it or **恢复保留** to undo rejection.
The most recent 20 complete episodes remain selectable after the next episode,
page refresh or restart. Older unreviewed recordings need explicit selection.

Reviews change selection, never raw files. Automatic keep is recorded as
`selection_source: automatic_on_save`, not human review. Collection uploads nothing.

## 4. Find and configure your data

The page displays the **Robot computer's** storage path and configuration file.
The dataset ID names the whole collection; each timestamped episode ID names
one recording.

```yaml
data:
  collection_root: .xlerobot/artifacts
  dataset_id: my-grasps
  dataset_root: .xlerobot/artifacts/datasets/my-grasps
  conversion_version: v1
  repo_id: local/my-grasps
```

```text
<collection_root>/datasets/<dataset ID>/
├── raw/<episode ID>/   manifest.json, data.npz, videos/
├── reviews/           editable keep/reject records
└── derived/<version>/lerobot/   conversion output
```

Set `collection_root` in `config/local.yaml` and restart collection to change
the storage root. `--config PATH` selects another local configuration.
Changing the dataset name on the page affects subsequent episodes only.
Point `data.dataset_root` at that full dataset path before conversion; when
editing the configuration also keep `data.dataset_id` consistent with it.
Configuration changes do not move old data.

## If preparation or following stops

Support both arms → **释放主从臂扭矩** → wait for confirmation → reposition →
**Reset / 重置状态** → Start. Reset clears the session, not recordings; it does
not home the arms or turn on their torque. The next Start prepares them.
The release operation also deactivates the shared right-arm/base bus, but does
**not** release the head or left arm.

A prominent banner identifies out-of-range initial joints. Do not force a
powered joint. If the issue is the head or left arm, stop and release the
corresponding hardware separately. If release or controller ownership cannot
be confirmed, stop the collection runtime before recovery.
See [collection troubleshooting](troubleshooting.md#collection).

## 5. Transfer, convert and train

Set `data.dataset_root` to the collection directory containing immutable `raw/`
episodes and editable `reviews/`. After optionally rejecting unwanted episodes in the Robot web UI, transfer
that whole dataset tree to the same repo-relative location on the GPU. Set the
`transfer` values in `config/local.yaml`, then use those values in the command
below (the example matches the documentation-only defaults):

```bash
# Robot computer; --ignore-existing prevents a rerun from replacing an episode.
rsync -a --checksum --ignore-existing --exclude reviews/ \
  .xlerobot/artifacts/datasets/xlerobot-glue-stick-grasp-30/ \
  operator@192.0.2.10:xlerobot_home_service_demo/.xlerobot/artifacts/datasets/xlerobot-glue-stick-grasp-30/
# Synchronize revised review labels as well; raw episodes remain unchanged.
rsync -a --checksum \
  .xlerobot/artifacts/datasets/xlerobot-glue-stick-grasp-30/reviews/ \
  operator@192.0.2.10:xlerobot_home_service_demo/.xlerobot/artifacts/datasets/xlerobot-glue-stick-grasp-30/reviews/
```

Then, on the GPU:

```bash
./tools/act convert --dry-run
./tools/act convert
./tools/act train --dry-run
./tools/act train --output .xlerobot/outputs/xlerobot-act-local-grasp-v1
```

The dry run reports accepted, rejected, missing-review, and missing-raw counts.
Conversion reuses `xlerobot_dataset_tools`, consumes only accepted immutable
episodes, and refuses to overwrite an existing derived version. The training
command pins ACT, wrist RGB `3 x 480 x 640`, six-value
state/action, chunk size 100, model dimension 512, feed-forward dimension 3200,
four encoder/one decoder layers, eight heads, latent 32, ResNet-18 ImageNet V1,
VAE on, and AMP off.

Resume an interrupted run from its `checkpoints/last`, including optimizer and
random state. Original dataset, batch size and policy settings are restored;
do not pass those settings again. `--steps` is the total target, not extra steps:

```bash
./tools/act train --resume --output .xlerobot/outputs/xlerobot-act-local-grasp-v1 --steps 6000
```

Omit `--steps` to finish the original target after an interruption. A completed
run needs a larger target. Weight-only downloads cannot resume training.

For a software-only pipeline check, use a separate output directory and a short
run (not a usable grasping model):

```bash
./tools/act train --output .xlerobot/outputs/act-smoke --steps 12 --batch-size 2
./tools/act train --resume --output .xlerobot/outputs/act-smoke --steps 16
./tools/act evaluate --checkpoint .xlerobot/outputs/act-smoke/checkpoints/000016/pretrained_model --output .xlerobot/outputs/act-smoke/qualification.json
```

Keep the demo checkpoint configuration unchanged during this check.

The released model was trained on exactly 30 yellow-glue-stick demonstrations.
The complete public dataset is
`lissajous/xlerobot-glue-stick-grasp-30` under CC BY 4.0. Shuttlecock
behavior is qualitative OOD evidence only.

## 6. Check the checkpoint and download

With the default 5,000-step run, training prints the concrete final checkpoint
path. Qualify that exact output and put the manifest beside it:

```bash
./tools/act evaluate \
  --checkpoint .xlerobot/outputs/xlerobot-act-local-grasp-v1/checkpoints/005000/pretrained_model \
  --output .xlerobot/outputs/xlerobot-act-local-grasp-v1/checkpoints/005000/pretrained_model/model-manifest.json
```

`evaluate` loads the local checkpoint, verifies the deployment structure, and
writes provenance plus a SHA-256 for every checkpoint file. It does not measure
real grasp success. Point `models.act_checkpoint` and `models.act_manifest` at
the checkpoint directory and generated JSON respectively; `tools/run gpu`
accepts either this local qualification schema or the fixed public-download
schema, and verifies every listed hash before serving.

The `download` operation uses the Apache-2.0 model ID
`lissajous/xlerobot-act-local-grasp-v1`, an immutable Hub revision, and
all fixed manifest hashes. Run `./tools/act download` on GPU to install the
published checkpoint; no Hub login is needed.
