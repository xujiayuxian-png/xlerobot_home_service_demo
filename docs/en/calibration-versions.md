# Apply, replace and restore calibration

`config/local.yaml` contains device/service settings and unit identity. Calibration
belongs to `.xlerobot/units/<unit>/`, where `<unit>` is `robot.unit_id` (matching
`calibration.unit`). Do not copy someone else's servo zeros or camera mounts to
another robot. This workflow does not replace models or checkpoints.

`draft/components/` contains new measurements; `versions/<id>/` stores immutable
versions; `active` selects one; `runtime/` contains generated, checksummed files.
The Demo does not read drafts. `draft/runtime/<workflow>/` is for calibration
capture only. Saving a hand-eye draft does not activate it.

## First installation

Complete your robot's [calibration workflow](calibration.md), then:

```bash
./tools/calibrate status
./tools/calibrate activate --version first-calibration
./tools/calibrate render
./tools/calibrate status
```

Require the intended active version and `runtime_matches_active: true` before
starting consumers. A new robot cannot use selective replacement to invent a
missing baseline: it requires an existing complete, valid active configuration.

## Replace selected measurements on the same robot

Stop Demo/collection/configuration consumers and record the old active version.
Drafts must come from the same physical robot and compatible predecessors.

```bash
./tools/calibrate replace --version calibration-v2 \
  --components servo head-camera right-handeye --dry-run
./tools/calibrate replace --version calibration-v2 \
  --components servo head-camera right-handeye
./tools/calibrate status
```

The first command previews without writing. The second validates the selected
drafts, preserves other active values, saves a new version, selects it and renders
runtime. `retained` does not mean newly tested. Use unique version names; existing
versions are not overwritten. Subsets are allowed but compatibility is not
automatically proven. Recalibrate downstream components after relevant mount/zero
changes. Old versions, drafts and captures remain available.

## Switch or restore

```bash
./tools/calibrate switch --version calibration-v2
./tools/calibrate switch --version ORIGINAL_VERSION
./tools/calibrate status
```

Replace ORIGINAL_VERSION with the recorded old ID. Switch selects **and renders**;
legacy `rollback` only selects and needs a separate render. On rendering failure
the tool attempts restoration; always inspect final status. No hardware is moved
and running nodes are not hot-updated. Restart consumers to load the new runtime.

## Actual effect

| Component | Runtime effect |
| --- | --- |
| servo | Follower/head decoding parameters in servos.yaml; not Leader |
| head-camera | Camera mounting transform in geometry.yaml |
| right-handeye | Jaw-to-Tag23 transform in geometry.yaml; full solution retained in transforms.yaml |
| Retained | Existing base geometry, lidar mount and grasp offsets |

Hand-eye `x` remains fixed-head camera/FK evidence; it does not directly replace
camera TF, calculate grasp offsets, retrain ACT or rewrite datasets. Leader has
its own [capture workflow](calibration.md).

For the same unchanged robot, a complete old snapshot can be adopted with
`./tools/calibrate import-runtime --input /path/to/snapshot --version adopted-runtime`.
It must contain geometry, servos, controllers, transforms and grasp_alignment YAML
files with provenance; see [follow-up instructions](calibration-followup.md).
Import checks configuration integrity, not new physical accuracy.

## Troubleshooting

- Incomplete draft: finish first-installation measurements, or use selective
  replacement on an existing unit; never fabricate missing evidence.
- Existing version: choose a new ID or switch to the existing version.
- Checksum error: preserve edits separately, then switch to a valid version;
  do not hand-edit runtime files in place.
- No visible change: inspect active/status, restart the correct consumers and
  do not confuse staged draft/runtime with the final runtime.
- Software acceptance: compare generated files/URDF, switch back and verify hashes;
  physical motion is not required to test configuration replacement itself.
