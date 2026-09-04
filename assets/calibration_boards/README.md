# Printable calibration boards

Use the PDF files for printing and keep the SVG files for inspection or custom
layout work. Print at **100% / actual size** with all “fit” or “scale to page”
options disabled, mount the page on a rigid flat backing, and verify one black
tag edge with a ruler before collecting samples.

| Target | Print file | Geometry |
| --- | --- | --- |
| Head camera | `head_4x4_ids_0-15_40mm.pdf` | AprilTag 36h11, 4×4 IDs 0–15, 40 mm tags, 12 mm gaps, A4 landscape |
| Right hand-eye | `handeye_tag23_60mm.pdf` | AprilTag 36h11 ID 23, 60 mm tag, A4 portrait |

Both PDF and SVG are deterministic vector outputs of
`ros2_ws/src/xlerobot_calibration_tools/scripts/generate_calibration_targets.py`.
The generator and runtime detector read the same
`ros2_ws/src/xlerobot_calibration_tools/config/targets.yaml` profile.

Regenerate all four files from the repository root:

```bash
python3 ros2_ws/src/xlerobot_calibration_tools/scripts/generate_calibration_targets.py \
  --output-dir assets/calibration_boards
```
