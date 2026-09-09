# Documentation images

`demo-ui.png`, `mapping-ui.png` and `collection-ui.png` are browser captures
of this repository's built HMI frontend. They use example identities and empty
sensor states in an isolated, loopback-only documentation preview. No ROS node,
motor driver, inference service, real map or camera was started for the captures.

The banner and camera placeholders are documentation-only overlays. These are
layout illustrations, not evidence of robot readiness or successful tasks.
The frontend itself is unchanged. Capture size: 1440 px wide; Chinese UI.

Project-owned screenshots are distributed under Apache-2.0 with the documentation.
Printable calibration PDFs/SVGs are in `assets/calibration_boards/` at the
repository root; do not print screenshots as calibration targets.

## Calibration pages

`calibration-head_camera.png`, `calibration-right_handeye.png` and
`calibration-hover.png` show the built calibration frontend with deliberately
illustrative data. Each image is labelled as a documentation preview, not measured
accuracy. Camera regions are replaced by labelled empty placeholders. No saved
unit calibration, camera frame, motion or real-device API is used.

Generate them with [capture_calibration.mjs](../artwork/capture_calibration.mjs).
It serves only the built HMI and documentation fixtures on an ephemeral loopback
port, rejects non-GET requests, and closes the browser/server afterwards. These
fixtures are artwork inputs, not a runtime mock framework or acceptance evidence.
Project-owned screenshots use Apache-2.0 with the documentation.
