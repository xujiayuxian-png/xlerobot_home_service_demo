# Reference hardware modification

This repository targets the one modified, two-wheel XLeRobot shown in the
[public fetch-and-deliver video](https://www.bilibili.com/video/BV1srNg6XEZj).
The video is also the public appearance and mounting reference. This is not a
compatibility promise for other XLeRobot layouts.

## Reference BOM

The table records what the software actually depends on. It deliberately does
not invent vendor SKUs, power ratings, or purchase links that were not retained
with the verified build.

| Quantity | Reference part | Purpose and reproducibility note |
| ---: | --- | --- |
| 1 | XLeRobot built around an IKEA RÅSKOG cart | Mechanical body; the repository contains the matching description meshes and frames. |
| 2 | SO-101-style follower arms with grippers | Five arm joints plus one gripper joint per side, using Feetech serial servos. |
| 1 | Two-axis Feetech pan/tilt head | Shares the left serial bus. |
| 2 | Feetech STS3215 wheel servos and wheels | Differential drive; wheel IDs are 9 and 10 on the right bus. |
| 2 | Independent Feetech USB serial adapters | One adapter owns the complete right bus and one owns the complete left bus. |
| 1 | SO-101-style right Leader arm and serial adapter | ACT demonstration collection only; normal inference and the demo never open it. |
| 1 | Intel RealSense D455 | Head RGB-D camera for VLM grounding and classical geometry. Use USB 3. |
| 1 | Sonix `USB2.0_CAM1`-class UVC camera | Fixed right-wrist observation at 640 x 480 for ACT. Use a stable video-device path. |
| 1 | LD06-style 2D lidar | Mapping, localization, and obstacle sensing; mounted upside down on the reference unit. |
| 1 each | Microphone and speaker | Chinese voice request and feedback path. |
| 1 | x86-64 robot computer | Ubuntu 24.04 and ROS 2 Jazzy; owns every motion device. |
| 1 | LAN NVIDIA computer | Ubuntu 24.04 or WSL2; the verified inference host has an RTX 3080. |

Brackets, wiring, power distribution, emergency-stop hardware, fasteners, and
the cart conversion are unit-specific mechanical work. Reproduce them for your
load and supply, then calibrate the result; do not infer their ratings from this
software repository.

## Bus and sensor topology

```text
robot computer
├── /dev/right_arm       Feetech bus: right arm/gripper IDs 1-6
│                                    wheel IDs 9-10
├── /dev/left_arm        Feetech bus: left arm/gripper IDs 1-6
│                                    head pan/tilt IDs 7-8
├── /dev/right_master_arm  right Leader IDs 1-6 (collection only)
├── /dev/lidar           upside-down LD06-style lidar
├── USB 3                head RealSense D455
├── stable /dev/v4l/...  right-wrist USB2.0_CAM1
└── audio input/output   microphone and speaker

LAN GPU computer
├── LM Studio / Qwen                 :1234
├── SAM2 + optional GPD proposals    :8765
└── ACT inference                    :8766
```

Servo IDs only need to be unique on their own physical bus. Reusing IDs 1-6
across the independent left, right, and Leader buses is intentional. Each
physical bus has exactly one runtime owner; do not run a second arm or base
driver against the same adapter.

The checked-in ROS description is the machine-readable reference for link
geometry, joint names, and the bus mapping. The values in its reference YAML
describe the recorded robot, not a calibration file to copy to a new unit.

## Mounting assumptions

- The D455 is below the pan/tilt chain and publishes the calibrated head camera
  frame. Preserve its view direction and USB 3 bandwidth.
- The wrist camera is rigidly fixed to the right wrist. Its pose and image
  orientation are part of the learned-policy observation contract.
- The lidar's reference transform includes a 180-degree roll. Changing the
  mount requires updating the description and repeating mapping validation.
- Wheel radius and separation are estimated for each unit by the base
  calibration; do not copy the reference numbers as measurements.
- Camera mounts and arm geometry are closed by head-camera, hand-eye, and
  grasp-alignment calibration. Even a visually identical rebuild needs its own
  bundle.

## Wiring and device names

1. Power down before changing a serial bus. Keep its servo type, voltage, and
   adapter compatible with the hardware you assembled.
2. Wire the right arm, gripper, and both wheels to the right adapter; wire the
   left arm, gripper, and head to the left adapter. Keep the Leader on its own
   adapter.
3. Before torque is enabled, scan each bus and confirm the exact IDs above,
   with no duplicates on that bus.
4. Create stable udev aliases matching `config/local.yaml`: `/dev/lidar`,
   `/dev/right_arm`, `/dev/left_arm`, and `/dev/right_master_arm`.
5. Set the D455 serial when more than one RealSense is connected. Configure the
   wrist camera through `/dev/v4l/by-id` or `/dev/v4l/by-path`, never a changing
   `/dev/videoN` index.
6. Route camera cables through the full head and arm range without tension or
   pinch points. Keep the D455 on USB 3 rather than sharing a constrained UVC
   link with the wrist camera.
7. Verify the microphone and speaker selected by `config/local.yaml`; no ALSA
   card number is portable between machines.

## First bring-up boundary

Run `./tools/doctor robot` and complete all calibration stages before enabling
motion. Have an operator beside the robot, clear the workspace, lift the drive
wheels for the initial direction check, and keep a physical stop method within
reach. This repository does not remotely test those assumptions.

After calibration, create and validate the local map only as explicit live
hardware operations:

```bash
./tools/run mapping --hardware --phase build
./tools/run mapping --hardware --phase validate
```

Run these phases separately, never concurrently. If using `--config`, use the
same file for both phases. In the web workspace:

To start mapping over, end teleoperation, wait until stationary, then confirm
“清除当前地图并重建” (clear the live map and rebuild). This resets live SLAM only;
saved maps, places and Demo configuration are untouched. Unsaved mapping cannot
be recovered. New scans rebuild the map; recheck or record place coordinates
before saving the replacement map.

1. Enable teleoperation and hold/drag the joystick to cover the site. Up/down
   drives forward/backward; left/right turns, including simultaneous driving
   and turning. Release sends zero; leaving the window ends teleoperation.
   The speed sliders set maximum speeds.
2. End teleoperation and wait for the robot to stop before recording a place.
   `table` is the final dock pose, facing the table: navigation first reaches
   a point 0.25 m behind it, then performs precise docking. Saved places appear
   on the map and in a selectable list for updating or deleting.
3. Save the current map after covering the entire site. Saving does not end
   mapping: save again if you continue exploring. The page shows the save time;
   a draft is not the active Demo map.
4. Stop build and start validate. Localize first, validate navigation to each
   place, then activate the draft. Replacing a map invalidates its navigation
   evidence; updating a place invalidates that place's evidence.
5. Activated files live under
   `<data.collection_root>/sites/<robot.site_id>/current/`. Set `site.map` and
   `site.places` in your local configuration to its `map.yaml` and `places.yaml`
   before the next Demo startup. A test site never silently replaces the old
   site's configuration.

Omitting the literal `--hardware` flag fails before any motor device is opened.
Follow the same boundary for every grasp and full-demo command.
