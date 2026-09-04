# XLeRobot reference description

`urdf/two_wheel_reference.urdf.xacro` is the single active model for the first
reference profile. It contains the differential base, both follower arms,
head, lidar, D455 mount, wrist-camera frames, and both physical-bus
ros2_control systems.

The model is adapted from the frozen prototype tag
`prototype-fetch-deliver-verified-20260709`. Reviewed geometry and joint limits
are centralized in `config/two_wheel_reference_geometry.yaml`. The runtime
servo calibration artifact is `config/two_wheel_reference_servos.yaml`.

Important invariants:

- `base_link` is the only URDF root and the differential-drive rotation center.
- The controller publishes `odom -> base_link`; there is no competing
  `base_footprint -> base_link` transform.
- `head_camera_link -> d455_head_camera_link` connects the project mount to the
  RealSense wrapper's prefixed base frame.
- RealSense owns calibrated internal optical transforms in hardware mode.
- Navigation uses wheel odometry and lidar; the reference profile has no IMU.
- `right_bus_system` alone owns right-arm servos 1-6 and wheel servos 9-10.
- `left_bus_system` alone owns left-arm servos 1-6 and head servos 7-8.
- Hardware and torque arguments default false; mock defaults true.

Validate without hardware:

```bash
xacro urdf/two_wheel_reference.urdf.xacro > /tmp/xlerobot.urdf
check_urdf /tmp/xlerobot.urdf
```
