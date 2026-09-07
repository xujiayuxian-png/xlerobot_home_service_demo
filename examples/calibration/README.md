# Calibration examples

Start with the [calibration guide](../../docs/en/calibration.md)
([中文](../../docs/zh-CN/calibration.md)). These files are small examples, not
calibration to activate on another robot.

- `head-camera/`: synthetic 12-pose solver replay with expected results.
- `right-handeye/`: anonymized 36-sample real Tag23 transform replay; no images.
- `base_measurements.yaml`: format for your commanded/actual base measurements.
- `grasp_alignment_measurements.yaml`: format for separate vision/FK and sag measurements.
- Other YAML files: component format examples; replace example values with measurements.

Run the two replays from the repository root, without hardware:

```bash
./tools/calibrate replay head-camera
./tools/calibrate replay right-handeye
```

Expected solver results do not establish your robot's physical accuracy.
