# Contributor notes

This repository reproduces one verified, two-wheel XLeRobot home-service demo.
Keep changes focused on that reference build; do not turn it into a generic
framework or a teaching platform.

- The public entry points are `tools/setup`, `tools/doctor`, `tools/calibrate`,
  `tools/act`, and `tools/run`.
- Real motion is allowed only through a command that explicitly includes
  `--hardware`. Commands without it must not open motor devices.
- Keep machine configuration in ignored `config/local.yaml` and secrets in the
  ignored `.env`. Never commit maps, recordings, logs, calibration for a real
  unit, model weights, or credentials.
- Remote VLM, segmentation, grasp-proposal, and ACT services return data only;
  controller commands remain on the robot computer.
- Use `apply_patch` for source edits. Build from `ros2_ws`, and add the smallest
  relevant software-only test for behavior changes.

