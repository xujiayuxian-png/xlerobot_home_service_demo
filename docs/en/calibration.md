# Unit calibration

[Documentation](README.md) · [中文](../zh-CN/calibration.md)

Start the [unified calibration workspace](calibration-workbench.md) with
`./tools/calibrate web --hardware` for servo, Leader, visual calibration, hover
metrology and version management on one page. Standalone commands below remain
available for diagnosis; they are not required for switching tabs.

The demonstrated grasping arm is the **right arm**. Hand-eye, hover validation
and ACT collection target it; head-camera calibration is shared sensing for this
demo. Servo calibration also covers the left arm, but a left-arm or dual-arm
demo has not been tested. Screenshots are in the [workspace guide](calibration-workbench.md).

## Prepare and follow the sequence

Keep any working calibration active while collecting new drafts. A saved sample,
a solved draft and an active runtime are three different things.

| Step | Prepare / measure | Complete when |
| --- | --- | --- |
| Servo | Exact bus IDs, joint directions, zero and raw travel | Joint set, ranges and limits pass validation |
| Head camera | Fixed rigid AprilTag 36h11 board, 4 × 4, IDs 0–15, 40 mm tags | Diverse head poses solve and save a valid draft |
| Right hand-eye | Rigid 36h11 Tag 23, 60 mm, attached to the gripper and observed by the D455 | Diverse arm poses solve and save a valid draft |
| Base | Tape-measured straight travel and rotation | Measured wheel radius/separation fit passes |
| Grasp alignment | Vision and FK observations of the same points, plus separate settled sag measurements | Compensation and sag save as separate fields |
| Activate / render | All five components for the same unit | `status` shows the intended active version and valid runtime |

The main dependency is `servo → head-camera → right-handeye`. Base measurements
can be done independently; all four structural components are needed before
grasp alignment. Stop demo, mapping or collection before each calibration workspace.

Use these printed targets at 100% size:
[head board PDF](../../assets/calibration_boards/head_4x4_ids_0-15_40mm.pdf) and
[Tag 23 PDF](../../assets/calibration_boards/handeye_tag23_60mm.pdf).
The head board has 12 mm gaps; check the actual printed dimensions with a ruler.

## Capture and save drafts

Copy `config/local.example.yaml` to the ignored `config/local.yaml` and set the
unit ID and stable device paths first. Live acquisition is routed through the
same public command and always requires explicit hardware consent:

```bash
./tools/calibrate capture servo --hardware
# Use the web page, finalize, stop it with Ctrl-C, then run the printed command.

./tools/calibrate capture head-camera --hardware
# Preview the board, then automatically capture, solve and save a head_camera draft.

./tools/calibrate capture right-handeye --hardware
# Automatically fit Tag 23 observations, freeze parameters, then validate held-out poses.
```

Each capture starts only one workflow and prints its local HMI URL plus the
exact follow-up command. Servo startup only connects the serial buses: it does
not enable torque or command motion. Head-camera calibration opens only the head
bus and holds only its two servos; neither arm, gripper nor wheel is commanded.
Hand-eye calibration holds only the right arm/gripper and head, without returning
to ready or accessing wheels/left arm. Motion begins only after explicit preview or automatic start.
Support the arm before releasing servo torque.
All raw captures stay below
`.xlerobot/units/<unit>/capture/` and remain outside Git.

Unfinished servo sessions restore automatically, paused after a process restart;
refreshing the page does not reset progress. Use `--resume` only to append
to the same head-camera/hand-eye session. Use `--fresh` after a mechanical
change; it moves the preceding workflow directory into the local `archive/`
instead of deleting it.

### Servo page: move the group, finish once

Work through **right arm → left arm → head**, without choosing individual
joints or repeatedly starting/stopping their recordings.
These are the robot's Follower arms. For the data-collection Leader, use
`./tools/calibrate capture servo --leader --hardware` (add `--fresh` to recapture).
It reuses this page with a single six-joint group and opens only the Leader bus.
This addition is software-tested, **not hardware accepted**. Results are saved
separately in `capture/calibration_work/leader_servo/result.yaml`; they do not
replace robot calibration or automatically change collection. Select explicitly
with `./tools/act collect --hardware --leader-calibration /path/to/result.yaml`.
Without this option, collection keeps the existing Leader calibration. Never
copy Follower zeros to a different Leader.

1. Select a group, support it and explicitly release that group's torque.
   Other groups and the wheels are untouched.
2. Choose the zero reference:
   - **Same robot, no servo reassembly or hardware-offset changes:** explicitly
     keep the verified active zero and remeasure only the ranges. The source
     is shown; previous ranges are never passed off as new measurements.
   - **New build or remeasuring zero:** use the page's URDF zero reference,
     position the whole group and capture once. This is the physical joint
     zero, not ready or an arbitrary mid-range pose.
3. Start group capture and manually move each joint, including the gripper.
   All joints record together; live position, min/max and coverage show what
   still needs movement.
4. Finish the group. Incomplete ranges are identified and remain available
   for further capture. The existing 60% range requirement still applies;
   do not force mechanical stops.
5. Once all three groups pass, save the result, then use the printed command
   to import it into a strictly validated draft.

Pause/continue preserves progress. Explicitly resetting a group clears only its
capture, not servo EEPROM or the previous active calibration. Local
`session.yaml` is checkpointed roughly once per second during recording; an
abrupt power loss may lose the last second. After a process restart, explicitly
release torque again before continuing manual capture. Final `result.yaml` is
written only when all groups pass. Saving never re-enables torque or activates
the calibration.

The interaction follows [LeRobot v0.5.1 group capture](https://github.com/huggingface/lerobot/blob/1396b9fab7aecddd10006c33c47a487ffdcb54b4/src/lerobot/robots/so_follower/so_follower.py).
This implementation preserves ROS physical-zero semantics: it does not invoke
LeRobot's motor homing writes or assume a full 0–4095 wrist range. Encoder wraps
and read failures are reported, never counted as successful range coverage.

### Head-camera page: preview, then automatic capture and solve

Import the passing servo draft first, then run
`./tools/calibrate capture head-camera --hardware`. Lay the complete 4×4 board
flat and secure it to the table. Its pose relative to the base and the table
height do not need measuring: the solver estimates the board pose as well.

1. Move to the preview pose using the page: pan=0, tilt=0.8 rad. Check that the
   complete board is visible: 16 detected tags and reprojection error below 1 px.
2. Confirm head motion and start automatic calibration. One shared pose file
   drives 25 views within pan −0.30…0.30 rad and tilt 0.60…1.00 rad. Each view is
   sampled only after motion completes and a fresh, stable detection is available.
3. Watch the annotated image, detection status, pose grid and valid sample count.
   Views without the complete board are explicitly marked skipped. Duplicate
   samples cannot fill the quota: at least 12 independent samples, observable
   motion and passing residuals are required to save the head_camera draft.
4. Inspect translation RMS/p95, rotation RMS and the saved path. This does not
   activate calibration or change the existing Demo runtime.

The sequence runs on the robot; refreshing the browser never restarts motion.
Pause cancels the current trajectory and keeps measurements for continuation.
A process restart also stays paused; use `--resume` for that same session.
Keep both board and base fixed throughout. After moving either or remeasuring,
pause and click **Archive and recalibrate** (归档并重新标定) on the page.
Confirmation archives samples, reports and a draft snapshot in a sibling
`.archive-<id>` directory. The new session remains idle: no motion, torque change,
draft deletion or activation. Confirm movement again before starting.
Changed predecessor calibration or pose configuration still requires restarting
with `--fresh` to load the new configuration. Never mix different setups.

Automatic solving uses the same strict solver as the CLI. A failed quality gate
shows its reason and preserves samples, without saving a passing draft or
relaxing thresholds. Hand-eye, base and grasp alignment remain necessary before
a new build can activate its complete bundle.

### Right hand-eye page: automatic capture and held-out validation

Finish the servo and head-camera drafts first, then run:

```bash
./tools/calibrate capture right-handeye --hardware
```

Rigidly mount 36h11 **Tag 23** on the right gripper's **fixed side**, with a measured
black outer-border size of **60 mm**. Do not bend or loosen the board. Its mounting
offset is solved jointly; manual measurement is not required. The table's 4×4
board is not used here. Keep the base stationary, clear the arm sweep and supervise locally.

The current mounting avoids the wrist camera and uses
`wrist_roll_offset_rad: -1.5707963267948966`: clockwise 90 degrees from the old
sampling poses when looking into the fingertips toward the wrist. Use `0.0` for
the old mounting. Changing this setting requires a fresh capture; never mix
samples from different mounts. Check visibility at the first pose before starting.

All 26 poses use different wrist-flex lifts of 0.12–0.24 rad (about 7–14°)
relative to the legacy seeds/midpoints, raising the fingertips about 17–37 mm
in reference URDF FK. Other joints are unchanged; fitting and held-out poses
remain distinct. The bounded capture flex envelope now extends to −0.98 rad;
robot joint limits are unchanged. This sweep completed one reference-robot capture
and held-out validation; it does not certify visibility or clearance for other installations. After a
pose-file change, stop the existing capture tool and restart with
`./tools/calibrate capture right-handeye --hardware --fresh`, which archives
the previous session. Do not resume samples collected with the old pose set.

Capture keeps receiving observations while loading/saving samples. If the latest
frame is stale or its TF has not arrived, it waits up to one second for synchronized
fresh evidence at the stationary pose. The 250 ms age limit is unchanged, both TF
pairs use the original image timestamp, and sample quality records `observation_age_sec`.

1. Check the D455 overlay and Tag 23 status. After confirming motion, optionally
   preview the first capture pose. This button moves both the right arm and head.
2. Start automatic calibration: 20 fitting poses, each captured only after arrival
   and fresh stable detections, with TF resolved at the image timestamp.
3. After coverage and fit-quality checks pass, freeze `fit.yaml` **before** moving
   through six different held-out poses. Validation never refits parameters.
   The revised 26-pose lifted-wrist sweep has completed one reference-robot validation.
4. Inspect fitting and held-out metrics separately, including per-pose errors.
   Both populations require translation RMS `<10 mm`, p95 `<15 mm`, and rotation
   RMS `<5°`; maximum errors are reported, not gated. Failed validation preserves
   its report and leaves the draft unchanged. Only a complete passing run saves
   the `right_handeye` draft; activation remains explicit.

The same ros2_control driver owns only right-bus IDs 1–6 and left-bus head IDs 7/8.
Wheels and the left arm are absent from this control description. Startup holds
measured positions, without a ready-pose motion. Keep the head at pan=0,
tilt=0.796136 rad and the gripper opening unchanged throughout capture.
Browser refresh observes the current run without restarting it. Pause cancels
the current trajectory and preserves samples; use `--resume` after a process
restart. Changed mounting/base placement or remeasurement uses the same web
**Archive and recalibrate** operation as head-camera calibration. Changed
predecessor calibration or pose configuration still requires a `--fresh` restart.

Local evidence lives under `.xlerobot/units/<unit>/capture/calibration_work/right_handeye/`:

| Files | Contents |
| --- | --- |
| `samples.yaml` / `progress.yaml` | All raw samples with pose IDs and resumable progress |
| `training.yaml` / `fit.yaml` | Only the 20 fitting samples and frozen transforms |
| `heldout.yaml` / `validation.yaml` | Six held-out samples, per-pose errors, thresholds and source hashes |

Do not refit all of `samples.yaml` and call its residuals independent validation.
The tag moves with the gripper: compare `base_from_camera × camera_from_tag`
against `base_from_jaw × jaw_from_tag`, not a stationary tag in the base frame.
This validates visual/FK closure on unseen poses, **not absolute fingertip or
grasp accuracy**. Later grasp alignment needs physical measurements at table
points; these residuals alone cannot uniquely separate vision bias and gravity sag.

### Other components and draft import

Base geometry is a tape-measure workflow. Either fill in
`examples/calibration/base_measurements.yaml`, or use the web worksheet started
by `./tools/calibrate capture base --hardware`. The worksheet does not drive
the base; execute each straight/turn trial under direct supervision and enter
the commanded and measured values.

The strict public solver/validator then stores one draft component at a time.
The live page deliberately does not activate a bundle. The paths below use
the example unit ID `demo-01`; replace it if you changed `robot.unit_id`:

```bash
./tools/calibrate servo --input .xlerobot/units/demo-01/capture/calibration_work/servo/result.yaml
./tools/calibrate base --measurements /path/to/base_measurements.yaml
# Or import the result printed by the base capture page:
./tools/calibrate base --input .xlerobot/units/demo-01/capture/calibration_work/base_geometry/result.yaml
./tools/calibrate head-camera --samples .xlerobot/units/demo-01/capture/calibration_work/head_camera/samples.yaml
# Automatic hand-eye capture already saves its draft and held-out report; do not refit all samples.
./tools/calibrate grasp-alignment --measurements /path/to/alignment.yaml
./tools/calibrate status
```

Each later capture must see the passing drafts that precede it. Render those
inputs into a staging directory without activating an incomplete bundle:

```bash
./tools/calibrate render --for head-camera      # requires servo
./tools/calibrate render --for right-handeye    # requires servo + head
./tools/calibrate render --for grasp-alignment  # requires all structural items
```

Base fitting uses commanded/actual tape-measure trials and can run in parallel
with the `servo → head-camera → right-handeye` chain. It becomes required when
rendering the grasp-alignment stage and when publishing the final bundle.

`tools/calibrate capture` performs this staged render automatically. The
rendered directory is `.xlerobot/units/<unit>/draft/runtime/<workflow>/`; its
manifest is marked `source: draft` and `final_demo_runtime: false`. The Demo
never reads this staging tree.

After every component passes validation, activate and render it:

```bash
./tools/calibrate activate
./tools/calibrate render
./tools/calibrate status
```

Plain `render` reads only the complete active bundle. It never promotes a
staged draft into the final sibling `runtime/` directory.

`tools/run demo --hardware` consumes only these rendered files and refuses to
continue if they are missing or invalid. It never reads component drafts
directly. Rollback is explicit:

```bash
./tools/calibrate rollback --version VERSION_ID
./tools/calibrate render
```

Replace `VERSION_ID` with the actual saved version you want to restore.

For a robot that already has a working calibration, `import-runtime` can
preserve its complete existing runtime in an immutable version. The input
directory must contain `geometry.yaml`, `servos.yaml`, `controllers.yaml`,
`transforms.yaml`, and `grasp_alignment.yaml`, with recorded provenance and
alignment marked `validation: existing_unit_runtime`. This checks executable
values and file integrity; it does **not** claim that old measurements passed
the new solver quality gates. Use it only for the same physical robot and
unchanged mounts. New builds should follow the measurement workflow above.

```bash
./tools/calibrate import-runtime --input PATH --version existing-unit-v1
```

Run the two solver fixtures without hardware:

```bash
./tools/calibrate replay head-camera
./tools/calibrate replay right-handeye
```

Passing replay proves the solver and file contract, not the calibration of your
robot. Do not commit `.xlerobot/`; it identifies one physical unit.

## Measure base geometry and grasp alignment

See [the follow-up guide](calibration-followup.md) for base measurement, inverted
lidar yaw and an offline reference tool, optional grasp alignment, a proposed
Tag 23 hover check, and selective `replace` / `switch` operations. New base and
alignment hardware acceptance is deferred; the instructions below remain optional.

The [base measurement template](../../examples/calibration/base_measurements.yaml)
contains example numbers, not calibration for your robot. Replace nominal wheel
dimensions and commanded/actual travel with your own measurements. The base
page is a form; it does not execute the straight/rotation trials for you.

For [grasp alignment](../../examples/calibration/grasp_alignment_measurements.yaml),
there is currently no capture page. Prepare a local YAML file with at least
three samples taken at the final mounting/head pose:

- `vision_xyz_m`: the point measured by vision, in `base_link`, metres.
- `fk_xyz_m`: the corresponding point expressed through robot FK in the same frame.
- `settled_z_shortfall_m`: separately measured downward sag, a nonnegative distance.
- `head_pose_rad` and `workspace_m`: the actual head pose and sampled region.

Do not fold the same sag into both coordinate compensation and the separate
sag measurement. The solver fits mean `FK − vision` separately from mean sag.
Use additional points not included in fitting to check the result.

## Read the quality result

Head camera needs at least 12 samples; hand-eye needs at least 20. Repeating
nearly identical poses is not adequate: the solver also checks pose coverage
and observability. Change orientation as well as position, keeping tags visible.

Hand-eye limits are translation RMS below 10 mm, p95 below 15 mm and rotation
RMS below 5°. Maximum error is reported, not used as a standalone rejection gate.
The exact thresholds live in
[quality.yaml](../../ros2_ws/src/xlerobot_calibration_tools/config/quality.yaml).
A low fit residual does not replace an independent physical alignment check.

After activating a new bundle, verify a few held-out targets and a controlled
grasp before treating it as a replacement for the previously working bundle.
No physical accuracy claim follows from replay alone.
