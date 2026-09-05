# ACT data-to-model workflow

The calibrated pregrasp and learned contact phase are shown in the
[two-stage ACT video](https://www.bilibili.com/video/BV18RK66JEdP).

The five public ACT operations follow one auditable path:

```text
collect on Robot -> convert on GPU -> train on GPU -> evaluate checkpoint
                                                        |
                                  manifest-pinned download after publication
```

## Collect

Connect the single right-arm Leader configured at `robot.devices.leader_arm`,
render the active calibration, then run on the Robot computer:

```bash
./tools/act collect --hardware
```

This launches the source collection workspace and can move the right Follower
arm. Review each immutable episode in the web UI. The reference release keeps
all raw recordings outside Git.

Open `http://<Robot address>:8080` (use the configured `demo.web_port`). Opening
the workspace does not start a demonstration. Keep the Leader's initial pose
inside its calibrated joint ranges, not folded against a mechanical stop.

1. For a first 10–15 second trial, select **通用手动** (manual). It starts from
   the Follower's current pose without detection or automatic pregrasp.
   **抓取模板** (pick) instead detects the object and moves the Follower above
   it using the Demo's shared pregrasp path.
2. Check the dataset ID and enter the actual object and instruction. Use a
   separate dataset for trials instead of mixing them into training recordings.
3. Click **Home / 开始采集**. Both modes automatically align the Leader to the
   Follower; do not drag it during alignment. Be ready to support it during
   the countdown, then move it only once `RECORDING` appears. The right
   Follower follows your demonstration.
4. Click **End / 正常结束并发布**, or let the duration expire. Following stops
   before camera videos and joint data are finalized. “Publish” here means a
   completed local episode, not an upload.
5. Once saving succeeds, choose **接受** (accept) or **拒绝** (reject). Only
   accepted episodes are converted; rejection does not delete raw recordings.
   The next Home starts a new episode without overwriting the previous one.

**Abort** cancels an interrupted trial and retains an incomplete recording;
it is not the normal save button. **状态机 dry-run** exercises phase transitions
only: it does not record data or establish camera/hardware readiness.
Raw episodes live at `data.collection_root/datasets/<dataset ID in UI>/raw/`,
with sibling `reviews/`. Point `data.dataset_root` at that dataset for conversion.

## Convert and train

The recorder writes `raw/` and immutable `reviews/` under the configured
`data.dataset_root`. After reviewing episodes in the Robot web UI, transfer
that whole dataset tree to the same repo-relative location on the GPU. Set the
`transfer` values in `config/local.yaml`, then use those values in the command
below (the example matches the documentation-only defaults):

```bash
# Robot computer; --ignore-existing prevents a rerun from replacing an episode.
rsync -a --checksum --ignore-existing \
  .xlerobot/artifacts/datasets/xlerobot-glue-stick-grasp-30/ \
  operator@192.0.2.10:xlerobot_home_service_demo/.xlerobot/artifacts/datasets/xlerobot-glue-stick-grasp-30/
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

The released model was trained on exactly 30 yellow-glue-stick demonstrations.
The planned complete dataset is
`xujiayuxian-png/xlerobot-glue-stick-grasp-30` under CC BY 4.0. Shuttlecock
behavior is qualitative OOD evidence only.

## Evaluate and download

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
`xujiayuxian-png/xlerobot-act-local-grasp-v1`, an immutable Hub revision, and
all fixed manifest hashes. It remains deliberately unavailable until the first
upload and revision are recorded; after that publication, run the documented
entry point `tools/act download`.
