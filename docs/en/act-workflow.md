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
