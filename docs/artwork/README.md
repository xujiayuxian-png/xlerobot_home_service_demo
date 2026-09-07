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
