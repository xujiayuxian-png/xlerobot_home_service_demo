# After calibration: base, lidar, tabletop checks and replacement

These are reproduction instructions, not claims of hardware acceptance. Base,
lidar and new grasp-alignment experiments are deferred for this release slice.
On the same unchanged robot, retain known values with provenance; never invent
zero-error measurements or use another robot's values as measured calibration.

## Base geometry

Use a flat non-slip floor, tape, a tape measure and a square, with normal load.
Mark the chassis forward direction and measure the same physical chassis point.
Do not use odometry as ground truth for odometry. Stop navigation/Demo before any
supervised motion. The existing base web tool is a measurement form, not an
automatic motion sequencer.

1. Drive slowly over approximately 1–2 m, repeating forward/backward trials.
   Record positive commanded/measured distance magnitudes. Correct wheel radius
   by `new_radius = old_radius * mean(actual_distance / commanded_distance)`.
2. **Apply that radius before collecting turn trials.** Repeat clockwise and
   counterclockwise turns, measuring full angle including under/overshoot (not
   angle modulo one revolution). Correct track width by
   `new_track = old_track * mean(commanded_angle / actual_angle)`.
   The solver assumes these sequential stages; do not mix old-radius turn trials.
3. Check held-out distances/angles. Report repeatability and real error. Curved
   straight runs, slip or asymmetric turns require mechanical investigation.

Fill [base measurements](../../examples/calibration/base_measurements.yaml) with
your measurements, then run `./tools/calibrate base --measurements /path/to/base.yaml`.
Example numbers are format examples only. New complete bundles use activate/render;
same-unit replacement can retain the old base without claiming it was remeasured.

## Lidar yaw: align the mount, not the map

The reference scanner is inverted: `sensors.lidar_roll` is π radians;
`lidar_yaw_deg` is horizontal mounting angle in degrees; `lidar_xyz` is in meters.
Positive yaw is counterclockwise about upward base Z. Do not duplicate corrections
in both the driver and TF. This check does not solve roll, pitch or translation.

Independently square the chassis to a flat wall, e.g. by equal wall distances from
two chassis reference points. Do not square it using the scanner being calibrated.
With the robot stationary, select one unobstructed wall spanning at least 0.5 m.
Export at least 20 finite points as a CSV with header `x,y`, in meters in the
original LaserScan frame: `x=r*cos(angle_min+i*angle_increment)`, `y=r*sin(...)`.
Exclude invalid/out-of-range returns, corners, legs, people and glass.

```bash
./tools/calibrate lidar-yaw --points /path/to/wall-scan.csv \
  --current-yaw-deg 186.442 --roll-deg 180 --wall-normal-deg 0
```

Replace 186.442 with the currently used yaw. A wall independently verified to be
directly ahead has normal 0°. Do not supply already-base-transformed points.
The offline tool handles inversion, fits a line normal, and prints a candidate
yaw, correction, RMS and span. It neither opens devices nor applies parameters.
It rejects short/noisy walls (RMS >10 mm), insufficient points and corrections
>15°. These checks are not an accuracy certificate. Repeat at other distances
and another independently aligned wall before accepting a correction.

Do not edit checksummed runtime files in place. Stop consumers, copy the five
runtime YAML files into a local staging folder, edit only
`geometry.yaml:sensors.lidar_yaw_deg`, record the measurement and old version in
`transforms.yaml:provenance`, then save the complete same-unit snapshot using
`./tools/calibrate import-runtime --input PATH --version VERSION`.
This selects/renders the snapshot; restart consumers to load its TF. Verify walls
and slow motion afterwards. Rebuilding the map is not a substitute for alignment.
No new lidar value or hardware test is applied by following this document alone.

## Tabletop accuracy: hover Tag 23 over board points

The [unified workbench](calibration-workbench.md) implements visual tabletop
alignment checking, not an absolute fingertip-accuracy certificate. Define a board point `(x,y)`, a
height `h` and a Tag orientation. The jaw goal follows
`base_from_board * board_from_tag_goal * inverse(jaw_from_tag)`.
The automatic workflow uses the center and X ±30 mm, with Tag center 200 mm above
the board and at least 50 mm table clearance for the checked arm/jaw/camera envelope.
It reports signed position and joint tracking errors and an unapplied total-offset
suggestion. Twenty frames at each point are not twenty independent arrivals;
repeatability and corrected accuracy need separate tests and an independent height check.

The target point itself may be occluded: infer it from other visible tags with
known board layout. Prefer at least three spatially spread, non-collinear tags
around the jaw silhouette while Tag 23 remains visible. Keep the head/base fixed.
If the board is hidden, validation waits for fresh eligible evidence and fails on
timeout; it does not substitute a frozen reference. Reacquire and replan after board
movement. Hover validation supports partial-board observations; head-camera
calibration still requires all 16 tags. Shared camera/model errors remain correlated;
independent physical references are required for absolute fingertip claims.

## Optional grasp alignment

New alignment experiments are deferred. Retain existing offsets with explicit
“not revalidated” provenance; do not silently zero them or infer gravity sag
from hand-eye residuals. If revisited, measure multiple actual tabletop endpoints,
use fixed compensation only for repeatable consistent offsets, and verify on
held-out points. Pose-dependent errors require investigation. Count independently
measured gravity sag only once. The existing
[measurement template](../../examples/calibration/grasp_alignment_measurements.yaml)
and solver remain optional tools, without a new mandatory experiment this round.

## Replace and restore a same-unit calibration

Stop Demo/collection consumers first. Replacement itself does not move hardware
or hot-update running nodes. Record the old active version from status:

```bash
./tools/calibrate status
./tools/calibrate replace --version my-calibration-v2 \
  --components servo head-camera right-handeye --dry-run
./tools/calibrate replace --version my-calibration-v2 \
  --components servo head-camera right-handeye
./tools/calibrate status
./tools/calibrate switch --version OLD_VERSION_ID
```

Use a unique new version ID. Replace validates selected drafts, retains the other
active values, versions the result and renders runtime. Switch selects and renders
a saved version. Retained base/lidar/grasp offsets are not newly accepted; a mixed
configuration still needs task-level verification. Downstream calibrations must
come from compatible predecessors. New robots still need a complete sourced bundle.

Servo replacement updates joint decoding; head-camera replacement updates the
camera mount; hand-eye `y` updates the jaw-mounted Tag 23 transform. Hand-eye `x`
is retained as evidence, **not substituted for camera TF or converted into grasp
offsets automatically**. Restart consumers to use runtime. Neither checkpoints
nor recorded datasets are automatically changed by configuration switching.
