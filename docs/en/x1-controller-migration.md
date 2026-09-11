# Rhino Pi X1 controller migration plan

[中文](../zh-CN/x1-controller-migration.md) · [Documentation](README.md)

Status: **planned; SSH access and read-only inventory completed, controller migration not validated**.
Inspection: 2026-09-12, source baseline `c3f85aa`. This is not an installation
guide or a change to the supported Ubuntu 24.04 x86_64 reference environment.

## Goal and development workflow

First replace the Robot computer, not the robot body or inference stack.
Keep ACT as the main backend and centroid/GPD on the same task/execution stack.
Do not simultaneously introduce AlohaMini, retrain ACT, move to Humble or create
a second motor runtime.

| Computer | Role |
| --- | --- |
| Laptop | Editor, browser and logs; preserve the working installation for comparison/rollback; no control dependency after migration |
| X1 | Device drivers, ros2_control, existing 50 Hz control loop, local ACT streaming executor, MoveIt, navigation, orchestration, web UI, calibration and collection |
| GPU host | Existing LM Studio/Qwen, SAM 2/GPD, ACT inference and training |

Robot-side KWS, CPU Whisper and person detection must also be ported and
profiled. Keeping heavy inference remote does not mean the X1 performs no
inference. Do not silently disable voice or person finding to pass acceptance.

Recommended workflow: **laptop editor over Remote SSH, with source, ARM64 builds
and tests on X1**. No X1 monitor is needed. AArch64 Linux is supported by Remote
SSH, but native editor extensions may have architecture restrictions; only SSH,
not the editor server, has been tested. See [VS Code prerequisites](https://code.visualstudio.com/docs/remote/linux).

The following alias was configured on the management laptop and tested:

```bash
ssh xlerobot-x1
ssh -o BatchMode=yes xlerobot-x1 'uname -m'
```

It is a local SSH configuration, not supplied by this repository. Reproducers
must configure their own address and public key. Passwordless SSH does not grant
passwordless sudo. Never copy the laptop's private key onto the X1.

Create an `adapt/x1-controller` branch on X1 during implementation; none was
created during inventory. Exchange commits using Git. Never synchronize x86_64
builds, virtual environments or machine settings into the ARM64 installation.
If an editor extension cannot run remotely, edit locally and transfer source in
one direction, then build/test on X1. Avoid editing two copies simultaneously.

## Observed environment

| Item | Observation | Consequence |
| --- | --- | --- |
| Platform | QCS8550 per local device notes; inspected `aarch64`, 6 CPUs | ARM64 dependencies and native build required |
| OS | AidLux Linux, Ubuntu 22.04.2, Python 3.10.12 | Does not match Noble/Jazzy |
| Kernel | Vendor `5.15.148-qki-consolidate-android13` family, PREEMPT | Preserve vendor kernel; timing not qualified |
| Resources | About 14 GiB visible RAM; 104 GiB root filesystem, about 81 GiB free | Start builds with two parallel workers |
| Tools | Git, GCC/G++, CMake, rsync present; ROS, colcon, Node/npm not found | Project cannot yet build/run |
| Isolation | Docker/Podman/nspawn absent; namespaces, cgroups, overlay and veth enabled | Some prerequisites only, no container execution tested |
| USB | 5 Gbit/s root controller; no robot cameras or serial controllers observed | Bandwidth, power and permissions untested |
| Drivers | Kernel configs enable ACM, CH341, CP210X, UVC and USB audio | Actual device behavior still untested |
| Ports | `filebrowser` occupies loopback 8080; 8081 also listening | Do not use default web port |
| Network/cooling | Wi-Fi uplink, separate bridged Ethernet LAN; fan-control service active | Preserve configuration; inspect DHCP/routes before wiring Ethernet |

Historical network addresses are not current. The board previously had a
gateway-address conflict; its Ethernet port is not assumed to be a normal DHCP
client. Old local notes describe an on-device Qwen2.5-VL experiment, but no 8888
listener was observed now and no model request was tested.

## Runtime decision gate

`tools/setup` currently requires Ubuntu 24.04 and x86_64 for both roles.
Jazzy supports Ubuntu 24.04 ARM64; architecture is not by itself the blocker.
See the [official ROS platform documentation](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Install-Binary.html).

1. Prefer a **vendor-validated Ubuntu 24.04 image**, if available with working
   USB, networking, cooling and future NPU support. No such image was established
   during inventory. Do not run a generic distribution upgrade or flash firmware.
2. Otherwise, propose **stock 22.04 host + minimal Ubuntu 24.04 ARM64 container +
   Jazzy**. This is an explicit exception to the project's original no-container
   scope and requires user agreement before implementation. No engine or image
   has been installed, no container started, and no kernel changed.
3. If isolation is rejected or fails on the vendor kernel, reconsider a separate
   Noble rootfs/chroot or vendor solution. Do not silently downgrade ROS or mix
   Noble packages into the Jammy host.

First validate the userspace, ROS communication and build without devices; then
test the required USB, serial and audio mappings. Containers share the host
kernel and do not guarantee driver compatibility or real-time performance.
GPU services never gain motor access. Keep hardware-disabled behavior intact;
do not publish unrestricted device/privileged access as the default. Validate
host udev, hotplug, networking and persistent paths without adding a release
bundle, automatic upgrades or a systemd delivery framework.

## Implementation checklist

- Separate Robot/GPU architecture checks in `tools/setup`; enable ARM64 for
  Robot only after validation. Leave the NVIDIA GPU environment unchanged.
- Audit Python 3.12 ARM64 wheels for sherpa-onnx, CTranslate2, ONNX Runtime,
  SciPy, PyTorch and the optional person extra. Preserve validated pins and
  update architecture-aware `doctor` checks; do not simply loosen versions.
- Rebuild Jazzy MoveIt, Nav2, ros2_control and device dependencies for ARM64,
  preserving the package boundaries and local executor behavior.
- Test D455 V4L2 first; consider a pinned librealsense RSUSB build if the vendor
  kernel is incompatible. Keep SDK/wrapper versions consistent. RSUSB is a
  candidate, not proof of X1 support. See [RealSense documentation](https://github.com/realsenseai/librealsense/blob/master/doc/installation_jetson.md).
- Preserve the existing 640×480 at 15 fps RGB/depth profile, aligned depth,
  no point cloud/IMU. Test direct USB3 before adding wrist camera, audio and
  serial devices; record resets, frame drops and load.
- Configure stable host udev aliases and actual device permissions. The current
  account is not in dialout; do not reuse laptop-specific numeric device indexes.
- Set X1 `demo.web_port: 18080` in ignored `config/local.yaml`; calibration uses
  its separate `--web-port 18080` argument. That port was free during inventory;
  recheck before use. Do not stop AidLux's filebrowser to free 8080. Run workspaces
  sequentially to avoid device/port ownership conflicts.
- Transfer same-robot active calibration, referenced versions, maps and required
  assets privately; regenerate runtime paths and verify checksums. Configure
  X1-to-GPU access separately: laptop-to-X1 SSH does not establish that link.

## Acceptance sequence

These are future tests, not completed results. Every wheel, head, arm or gripper
test still requires explicit authorization for that test.

| Stage | Work | Completion evidence |
| --- | --- | --- |
| P0 | Access and inventory | Completed: key login and environment checks |
| P1 | Agreed runtime, ARM64 dependencies/build, HMI npm ci/test/build, software tests | Repeatable device-free build and five-entry contracts; expected missing-device doctor findings distinguished from failures |
| P2 | Connected cameras/lidar/audio and GPU HTTP, no motor movement | Stable data; target a 30-minute capture test and record actual results |
| P3 | Authorized buses, base, head, right arm/gripper, ACT executor | Preserve 50 Hz and trajectory semantics; measure p95/p99 cycle timing, timeouts, load and temperature; do not hide jitter by loosening watchdogs |
| P4 | Existing active calibration/map, ACT then centroid/GPD, voice and HMI | Eight-stage demo with no laptop ROS/control dependency; at least one controlled functional task per backend, no success-rate claim |
| P5 | Mapping, collection, conversion/training workflow, calibration UI/version management | Same formats and configured collection timing; training remains on GPU; changing computers alone does not imply recalibration |
| P6 | Publish measured environment and limitations | Only passed combinations become supported configurations |

Test control first at low load, then with cameras, voice, person finding and
web activity. A 50 Hz loop has a 20 ms period; PREEMPT alone is not deadline
evidence. Compare measurements with the laptop baseline and existing timeout
constraints. Verify independence with the laptop disconnected and another
browser device available to the on-site operator.

## Assets and rollback

Preserve the laptop's working revision and local state. Copy only required
active versions, maps/places, models and audio; preserve symlinks and check
absolute paths. Do not copy environments/caches wholesale or commit `.env`.
Same-robot calibration can be retained only if the relevant physical mounts
are unchanged. Moving cameras, tags or mechanics requires rechecking affected
calibration. Do not activate experimental drafts as part of the computer swap.
Use `status`/`render` and the [replacement guide](calibration-versions.md).

To return to the laptop, stop all X1 robot nodes before reconnecting devices.
Never allow both computers to own the motors. If hardware geometry changed,
the laptop's previous calibration is not automatically valid either.

## Later: on-device AI

After controller acceptance, evaluate CPU/NPU person detection, speech or VLM
one at a time, measuring quality, latency and control load. Historical AidLux
Qwen2.5-VL experiments are not a drop-in replacement for the configured Qwen3-VL
contract. Keep ACT/SAM 2/GPD and training on the GPU first. Treat model migration
and AlohaMini body changes as separate projects.

Next decision: agree on the X1 runtime exception before P1. This plan does not
authorize disconnecting the laptop, installing the full stack or moving motors.
