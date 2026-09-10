# Documentation images

`demo-ui.png`, `mapping-ui.png` and `collection-ui.png` (Chinese), and their
`*-en.png` counterparts (English), are browser captures
of this repository's built HMI frontend. They use example identities and empty
sensor states in an isolated, loopback-only documentation preview. No ROS node,
motor driver, inference service, real map or camera was started for the captures.

The banner and camera placeholders are documentation-only overlays. These are
layout illustrations, not evidence of robot readiness or successful tasks.
The frontend itself is unchanged. Capture size: 1440 px wide. English documents
use the English captures; Chinese documents use Chinese captures from the same
frontend build. The
English collection form contains explicitly entered example text, not an
automatic translation of a real dataset or instruction.

Reproduce with [capture_ui.mjs](../artwork/capture_ui.mjs): pass `en` for English
or `zh` for Chinese. It uses only static assets and isolated fixture responses.

## Screenshot index

These pairs are shared by the root READMEs and the corresponding workflow
guides. Refresh both columns together; the filenames without a language suffix
are Chinese, not language-neutral images.

| Page | English screenshot | Chinese screenshot | Guides |
| --- | --- | --- | --- |
| Demo | [English](demo-ui-en.png) | [中文](demo-ui.png) | [EN](../en/demo.md) / [中文](../zh-CN/demo.md) |
| Mapping | [English](mapping-ui-en.png) | [中文](mapping-ui.png) | [EN](../en/mapping.md) / [中文](../zh-CN/mapping.md) |
| ACT collection | [English](collection-ui-en.png) | [中文](collection-ui.png) | [EN](../en/act-workflow.md) / [中文](../zh-CN/act-workflow.md) |
| Head camera | [English](calibration-head_camera-en.png) | [中文](calibration-head_camera.png) | [EN](../en/calibration-workbench.md) / [中文](../zh-CN/calibration-workbench.md) |
| Right-arm hand-eye | [English](calibration-right_handeye-en.png) | [中文](calibration-right_handeye.png) | [EN](../en/calibration-workbench.md) / [中文](../zh-CN/calibration-workbench.md) |
| Hover accuracy | [English](calibration-hover-en.png) | [中文](calibration-hover.png) | [EN](../en/calibration-workbench.md) / [中文](../zh-CN/calibration-workbench.md) |

The URDF renders, grasp-route diagram and servo-zero reference already have
English labels and are shared across languages. The author's Xiaohongshu card
is the original supplied image, including its Chinese text and scannable code.

Project-owned screenshots are distributed under Apache-2.0 with the documentation.
Printable calibration PDFs/SVGs are in `assets/calibration_boards/` at the
repository root; do not print screenshots as calibration targets.

## Calibration pages

`calibration-head_camera.png`, `calibration-right_handeye.png` and
`calibration-hover.png`, plus their `*-en.png` counterparts, show the built calibration frontend with deliberately
illustrative data. Each image is labelled as a documentation preview, not measured
accuracy. Camera regions are replaced by labelled empty placeholders. No saved
unit calibration, camera frame, motion or real-device API is used.

Generate them with [capture_calibration.mjs](../artwork/capture_calibration.mjs),
passing `en` or `zh`. English captions, camera placeholders and fixture messages
are included; the script rejects untranslated text in English captures.
It serves only the built HMI and documentation fixtures on an ephemeral loopback
port, rejects non-GET requests, and closes the browser/server afterwards. These
fixtures are artwork inputs, not a runtime mock framework or acceptance evidence.
Project-owned screenshots use Apache-2.0 with the documentation.
