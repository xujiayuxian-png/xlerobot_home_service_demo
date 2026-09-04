# Third-party notices

## XLeRobot model assets

The files under
`ros2_ws/src/xlerobot_description/meshes/xlerobot_original/` and portions of
the description macros are adapted from the XLeRobot project. They were
imported through the frozen prototype from its XLeRobot checkout at commit
`51ca0ec31bdb48713b94bacdba828bf8d889296b`.

- Source checkout: <https://github.com/xujiayuxian-png/XLeRobot>
- Upstream project: <https://github.com/Vector-Wangel/XLeRobot>
- License: Apache License 2.0

Project-specific wheel, sensor, arm-mount, and camera calibration values are
documented separately and are not presented as upstream defaults.

## SCServo Linux SDK

`third_party/SCServo_Linux` is a pinned Git submodule. Its source and license
remain in that submodule; `xlerobot_feetech` provides the project-owned ROS 2
boundary around it.
