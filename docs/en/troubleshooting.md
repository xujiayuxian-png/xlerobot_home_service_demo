# Troubleshooting

[Documentation](README.md) · [中文](../zh-CN/troubleshooting.md)

Start with the **first error** and the configuration used for that command.
Later controller/lifecycle errors may be consequences. Doctor is read-only:
it checks configuration, environments, paths, hashes and HTTP services without
opening cameras or motors.

| Symptom | What to do |
| --- | --- |
| Missing/invalid configuration | Copy `config/local.example.yaml`; keep `schema: xlerobot_demo/v1`. Unit IDs and dataset paths must agree. |
| Missing calibration/runtime | Finish the five components, activate, render and check status; see [calibration](calibration.md). |
| Missing map or `table` | Follow [build → save places → validate → activate](mapping.md), then set both site paths. |
| LM Studio unreachable | Load the recorded model and enable LAN serving; check `/v1/models` from Robot and the Windows/WSL firewall. |
| Port 8765 fails | Check classical Python environment and SAM 2 with `tools/doctor gpu`. GPD additionally needs `tools/setup gpu --with-gpd`. |
| GPD fails but centroid works | Read the reported GPD candidate/workspace failure; GPD intentionally never silently falls back. |
| Port 8766 fails | Check CUDA, checkpoint directory, manifest and token with doctor. |
| ACT unauthorized | Put the same `XLEROBOT_ACT_TOKEN` in both computers' `.env` files. |
| Public model download unavailable | Check [asset availability](assets.md); pending public release is not a machine setup failure. |
| Camera missing/frozen | Stop the workspace before reconnecting. Use USB 3 for the D455; check cable, bandwidth and stable wrist-camera path. |
| Device alias missing | Fix the stable udev alias; do not replace it with a changing `/dev/ttyUSB*` index. |
| Voice does not hear/speak | Check input/output selection, KWS/Whisper files and locally generated prompts; see [install](install.md). |
| Web page missing/stale | Re-run Robot setup to rebuild the frontend/workspace, then restart and reload the browser. |
| Live command refused | Pass the literal `--hardware` only when ready for the corresponding real operation. |

## Collection

- **Initial pose warning:** read the named joint, actual angle and allowed range.
  Support both arms, release torque, reposition, Reset, then Start.
  The release button does not release the head/left arm.
- **Home appears to wait:** preparation holds Leader torque until Home. After
  Home, wait for `RECORDING`, not just `STARTING_RECORDING`, before moving.
- **End but the arm still follows:** intentional; put the object down and return.
  Only recording stopped. Use release torque to stop following and reposition.
- **No acceptance button:** complete new episodes are kept automatically.
  Use reject/restore in the recent episode list; raw files remain.
- **Conversion sees no selected data:** check the actual dataset path, review
  status and whether you transferred `reviews/` after changing selection.
  A converted LeRobot dataset goes straight to training, not raw conversion.
- **Teleop stops unexpectedly:** invalid/missing input, stale feedback and
  lease expiry stop following; correct the reported issue and start a new episode.
  Finite Leader positions are clamped to Follower limits, so ordinary boundary
  contact alone should not cancel collection.
- **Reset/release fails:** do not force a powered arm. Stop the collection
  workspace and inspect the first controller/hardware error before restarting.

### Leader timing diagnostic

The upstream message `High execution jitter or mean error` combines two checks.
Inspect mean and standard deviation rather than assuming it proves jitter.
The reference 50 Hz, six-servo Leader read normally takes about 1.37 ms.
Its hardware mean-execution budget is 2 ms warning / 4 ms error; standard
deviation thresholds remain 100 / 200 µs. Periodicity, feedback and lease checks
remain enabled. See [Jazzy diagnostic definitions](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html#parameters).
Do not keep widening thresholds to hide new communication faults.

## Stop and recover

Ctrl-C the foreground tool before starting another workspace. If shutdown
reports failed torque release or a serial timeout, support the arms and switch
off motor power; a stopped process does not prove the servos acknowledged release.
Keep your recordings and calibration drafts when troubleshooting.
