# X1 native Ubuntu 22.04 migration

[中文](../zh-CN/x1-controller-migration.md) · [Documentation index](README.md)

Status: **P1 native installation, build and software acceptance passed on X1;
physical motion has not been accepted**. Updated 2026-09-12.
The user's native Ubuntu 22.04 requirement supersedes the earlier Noble container proposal.

Measured results: the public setup built all 20 ROS packages. After workspace
testing and targeted reruns, colcon reports 1021 tests, zero errors/failures and
45 skips. HMI's 13 frontend test files / 96
tests and production build passed. Calibration passed 161 tests, collection 58;
the additional hover image path passed a separate 12-test run. Real Nav2 plugins
and its behavior tree activated with synthetic sensors. Mock bus/Leader
lifecycle checks passed. CPU person inference and KWS/Whisper int8 model loading
passed; 22 voice prompts were generated. All five help commands, shellcheck and
the no-hardware Demo rejection passed.

Doctor passed ROS, dependencies and models. Its 18 errors and one warning are
remaining device, calibration/map, GPU endpoint/token prerequisites. Version
records and logs live under ignored `.xlerobot/environment`. Software tests do
not qualify physical camera/audio acquisition, 50 Hz timing or the full demo.

## Platform and ownership

The X1 retains AidLux / Ubuntu 22.04 ARM64, its vendor kernel and Python 3.10,
using ROS 2 Humble. No container, OS upgrade, or Noble packages are needed.
[ROS lists Jammy ARM64 as a Humble platform](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html).
The original Ubuntu 24.04 x86_64 / Jazzy reference remains available.

X1 owns drivers, ros2_control, the local ACT executor, MoveIt, navigation, task
orchestration, HMI, voice, person detection, calibration and collection. The GPU
continues to own LM Studio, SAM 2/GPD, ACT inference and training. Remote services
return data only; motor commands remain local to the robot computer.

## Native installation

From a recursive checkout on X1:

```bash
./tools/setup robot --system-only
./tools/setup robot --with-person-detector --with-kws-model --generate-voice-prompts
```

A new checkout also needs ignored `config/local.yaml` copied from
`config/local.example.yaml`, with web port 18080 and real device/GPU settings.
Keep secrets separately in `.env`.

The first command installs system/ROS dependencies with sudo. The second installs
the Python environment, verifies model downloads, tests/builds HMI and builds the
workspace. Setup never starts robot nodes or opens motor devices.
It selects `/opt/ros/humble` only for Jammy ARM64; GPU retains its original guard.
The Jammy ROS apt repository uses its verified Open Robotics signing key.
The official HTTP apt endpoint relies on signed indexes and package hashes.

Node 22.22.0 is checksum-verified and installed under ignored `.xlerobot/vendor`;
Jammy's Node 12 cannot build Vite 6. Python uses `.venv/robot` with ROS system
packages. Colcon runs through that interpreter so Python nodes retain access to
voice dependencies. Builds process packages sequentially with two compiler jobs;
`XLEROBOT_BUILD_WORKERS` controls the latter.

`requirements/robot-x1.txt` pins ONNX Runtime 1.23.2 for Python 3.10,
NumPy 1.26.4 and OpenCV 4.11.0.86 for Humble's NumPy 1.x cv_bridge ABI.
`robot-person-x1-agpl.txt` selects ARM64 CPU PyTorch wheels explicitly: current
PyPI ARM64 torch also installs CUDA dependencies. Reference pins remain intact.
Resolved versions are recorded under ignored `.xlerobot/environment`.
Ultralytics is AGPL; KWS model terms remain unresolved, and both retain explicit
setup flags. Model files stay outside Git and are checked against the manifests.

## Existing AidLux package repair

This X1 arrived with unpacked/unconfigured libkmod2 29-1ubuntu1.1 and kmod
29-1ubuntu1. A normal apt repair attempted to overwrite AidLux mod-blacklist's
`/etc/modprobe.d/blacklist.conf`. Restoring libkmod2 to matching 29-1ubuntu1
resolved the dependency while preserving that vendor file. No forced overwrite,
vendor package removal or kernel change was used. This machine-specific repair
is deliberately not automated in setup. Inspect `dpkg --audit` and simulate apt
repairs before addressing a recurrence.

## Compatibility boundaries

Hardware initialization supports Humble HardwareInfo and Jazzy component
parameters, sharing the same bus gates, calibration and torque implementation.
Calibration supports modern OpenCV ArucoDetector and uses the same centered
corners with iterative solvePnP when the old single-marker pose helper is absent;
synthetic tests check rotation, translation and reprojection error.
The Leader controller supports Humble's void-returning synchronous writes while
retaining lease expiry and torque-off lifecycle handling. Lease parameters use a
spawner parameter file. Public tools select the native ROS installation; tests
resolve controller_manager through the package index.

Humble's SLAM is a regular node and lacks the newer live Reset service. Mapping
and saving remain available; clearing the live map requires stopping and
restarting the mapping session. HMI explicitly rejects unsupported live resets,
preserving the preview and saved assets. Clear only removes interactive edits
and is not substituted for Reset.

Humble loads a small Nav2 parameter override and a BT.CPP 3 behavior tree for
plugin names, progress checking and collision point thresholds. Its action
results lack Jazzy's structured errors. Callers still require SUCCEEDED and a
non-null result; failed BackUp stops recovery without blind forward retry.
Errors without detailed codes are reported as generic backend failures. The
Humble recovery tree cannot use Jazzy's error-code predicates, so physical
recovery behavior must be requalified. Local cancellation waits and deadlines
remain in place. Jazzy-only controller diagnostics do not become timing evidence
on Humble; measure 50 Hz loop p95/p99 and load explicitly.

## Configuration and acceptance

Keep machine settings in ignored `config/local.yaml`, secrets in ignored `.env`.
The X1 local web port is 18080; preserve AidLux filebrowser on 8080. Calibration
needs its own `--web-port 18080`. Alternate demo, mapping, collection and
calibration sessions to avoid resource contention.

Before physical testing, configure GPU URLs, X1-to-GPU SSH and the shared ACT
token, serial/video aliases, D455 serial and audio devices. Privately migrate the
same robot's complete active calibration, map and places; render/check them on
X1. Never copy x86 build/install/venv directories or numeric device assignments.
Keep D455 RGB/depth at 640x480/15 fps, aligned depth, no point cloud or IMU.
See [calibration versions](calibration-versions.md) for asset activation.

```bash
source /opt/ros/humble/setup.bash
source .venv/robot/bin/activate
cd ros2_ws
source install/setup.bash
python "$(command -v colcon)" test --executor sequential --event-handlers console_cohesion+
colcon test-result --verbose
cd ..
./tools/doctor robot
```

P1 requires workspace build, HMI tests/build, software tests and all five public
help commands. Missing device/calibration/map/GPU doctor errors identify remaining
commissioning work, not a build failure, and still block live operation.
P2 checks peripherals (target 30-minute acquisition). P3 checks each authorized
motion and 50 Hz timing under progressive load. P4 qualifies the full ACT demo,
centroid/GPD, voice and HMI independently of the laptop. P5 qualifies mapping,
collection, conversion, GPU training dispatch and calibration replacement.
All real motion requires explicit `--hardware` and authorization for that test;
installation permission is not permission to move the robot. Preserve the laptop
for rollback and stop the other controller before switching hosts. NPU inference
and mechanical changes remain separate follow-up work.
