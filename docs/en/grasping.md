# Two grasp routes

Both routes use the same calibrated robot description, object request,
pregrasp planning, robot-local controller path, and grasp verification. The
choice is carried explicitly in `ExecuteTask.grasp_backend`; there is no silent
fallback from one route to another.

ACT is the main demo backend; centroid and GPD are alternative grasp backends,
not separate robot stacks. All use the same active geometry/servo calibration,
timestamped TF, grasp alignment and robot-local execution workspace. Classical
perception does not require another head-camera or hand-eye solver export.
Calibration sample bounds record where the fit was measured; they are not a
second runtime workspace. Same-unit imported calibration retains its provenance
without claiming newly measured accuracy, particularly at contact height.

GPD runs in the GPU computer's Ubuntu/WSL environment, alongside SAM2 on port
8765. Its native checkout/build is local state under `.xlerobot/vendor/gpd`;
`./tools/setup gpu --with-gpd` builds the pinned revision for a clean install.

## Classical RGB-D route

The classical service on port 8765 combines VLM grounding, prompted SAM 2
segmentation, aligned D455 depth, table/object geometry, and a top-grasp target.

- `centroid` selects the deterministic object centroid/top geometry path. It is
  the smallest dependency and best debugging baseline.
- `gpd` asks the pinned GPD backend for candidates and applies the same
  XLeRobot workspace/top-grasp filtering before execution.

The service returns perception proposals only. The ROS manipulation side still
owns planning and execution.

## Hybrid ACT route

`act` first uses calibrated vision and MoveIt to reach a deterministic
pregrasp. It then sends the measured six-joint state and the checkpoint's wrist
image to port 8766. The returned 100-step chunks are proposals; the local
streaming executor enforces joint ordering and command bounds before the normal
controller path.

The exact model contract, runtime pins, and checkpoint hashes are in
`../../assets/models/act-local-grasp.md` and `manifest.yaml`. It was trained
only on 30 yellow-glue-stick demonstrations. Shuttlecock and other objects are
qualitative OOD demonstrations with no success-rate or generalization claim.
The Apache-2.0 weight upload is pending, so use the verified local checkpoint
until an immutable Hub revision is recorded.

## Running a comparison

Start the GPU services, start the robot stack once, and submit one explicitly
named backend per trial:

```bash
# GPU computer
./tools/run gpu

# Robot computer, terminal 1
./tools/run demo --hardware

# Robot computer, terminal 2
./tools/run grasp centroid --hardware
./tools/run grasp gpd --hardware
./tools/run grasp act --hardware
```

Use the same object placement, active calibration, source place, and lighting
when comparing routes. Record success/failure and latency separately; do not
treat fallback behavior as a success for the requested backend.
