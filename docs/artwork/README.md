# README artwork

`robot-hero.png` and `robot-detail.png` are offline visualizations of the
checked-in `two_wheel_reference.urdf.xacro`: 42 visual elements, using the 24
existing STL meshes plus URDF primitives. Link transforms, mesh scales and
joint axes come from that description. The right-arm display pose uses the
checked-in pregrasp seed; other display angles are within the URDF limits.
Display poses are **not** motion commands or collision-checked plans.

Materials, lighting and camera composition are styled for the README. These
are not photographs, a simulation test, or calibration evidence. Frames with
no visual geometry do not gain an invented sensor body. Use the hardware
guide and linked real-robot videos for the actual build and appearance.

The XLeRobot mesh provenance and Apache-2.0 terms are recorded in
[third-party notices](../../THIRD_PARTY_NOTICES.md#xlerobot-model-assets).
`grasp-routes.svg` is an editable, project-owned vector diagram.

## Re-render (documentation maintainers only)

This is independent of robot setup. It needs Node.js/npm, Chrome, and the
ROS Jazzy `xacro` Python module. Rendering dependencies stay in the ignored
local directory; they are not added to the HMI or robot runtime.

From the repository root:

```bash
npm install --prefix .xlerobot/readme-art --no-audit --no-fund three@0.170.0 playwright@1.49.1
source /opt/ros/jazzy/setup.bash
python3 docs/artwork/render_robot.py
```

Set `CHROME_BIN` if Chrome is not at its usual Linux location. The script
briefly serves only artwork, meshes and render dependencies on loopback,
then captures two 1600 × 900 images and closes the browser and server.
No ROS nodes are launched, no device is opened, and no hardware is needed.
Typography or antialiasing can vary slightly with the installed browser/fonts.

Edit layout, camera, lighting and display pose in `robot.html`; edit the route
diagram directly in `../images/grasp-routes.svg`. Keep the two READMEs in sync.

## Servo zero reference

The servo calibration page uses
[`calibration-zero.png`](../../ros2_ws/src/xlerobot_hmi/web/public/calibration-zero.png), rendered
from the same checked-in URDF at **exactly q = 0 for every movable joint**.
Two orthographic views show the arm/head assembly. This is the mechanical
zero reference, not a ready pose, LeRobot's approximate midpoint, or a claim
that the physical robot has already been calibrated. Joint transforms and
meshes are unchanged; only the materials and framing are styled.

```bash
source /opt/ros/jazzy/setup.bash
python3 docs/artwork/render_robot.py --view zero
```

This writes only the 1400 × 850 calibration image, leaving the README hero
and detail images unchanged. `capture_zero.mjs` verifies that every movable
joint is set to zero before capture; it never connects to ROS or hardware.

## UI and calibration screenshots

Build the HMI (`npm run build` in `ros2_ws/src/xlerobot_hmi/web`), install the
documentation Playwright dependency as above, then run from the repository root:

```bash
node docs/artwork/capture_ui.mjs en
node docs/artwork/capture_calibration.mjs en
# Refresh both languages from the same frontend build.
node docs/artwork/capture_ui.mjs zh
node docs/artwork/capture_calibration.mjs zh
```

These capture the Demo, mapping and collection pages, plus head-camera,
right-arm hand-eye and hover-result panels at 1440 px viewport width. English
files have an `-en.png` suffix and do not overwrite the Chinese captures. The
scripts select the actual frontend language; they do not paint translated text
over screenshots. English collection form values are explicitly entered examples.

All numbers are illustrative, with an explicit banner and no camera connection.
No `.env`, local configuration, saved unit data or ROS service is used. The
servers bind to loopback, reject non-GET requests and close after capture.
Fixtures are confined to these documentation scripts. English captures are
checked for untranslated Chinese text (the **中文** language-switch label is
intentionally retained). Keep README and language-specific image references in sync.
Refresh both languages whenever the UI changes, including screenshots used by
the Chinese guides; do not leave those on an older UI build. Run
`python3 tools/lib/test_public_tools.py` to check local image links throughout
the repository's own documentation and the language of UI image references.
