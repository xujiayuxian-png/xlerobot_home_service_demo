# XLeRobot Home Service Demo

[中文](README.zh-CN.md) · [Documentation](docs/en/README.md)

**A slightly modified XLeRobot that hears a request, fetches an object from a
table, and delivers it to a person—with the code and tools to reproduce it.**

[Complete robot demo](https://www.bilibili.com/video/BV1srNg6XEZj)
· [ACT grasping](https://www.bilibili.com/video/BV18RK66JEdP)
· [Web console](https://www.bilibili.com/video/BV1GSK66XEqf)

The focus is **calibration and two grasp routes** on one two-wheel reference
robot, not a general framework or a course:

- **Hybrid ACT, the main demo:** RGB-D target → calibrated MoveIt pregrasp →
  wrist-image ACT action chunks → robot-local execution.
- **Classical geometry:** VLM + SAM 2 + RGB-D → centroid or GPD top-grasp plan →
  MoveIt execution. GPD runs on the GPU computer and never silently falls back
  to centroid.

Both routes use the same robot stack, calibration and task flow. The default
is `act` + `羽毛球` (shuttlecock), with voice and web enabled. The checkpoint
was trained on **30 yellow-glue-stick demonstrations only**; shuttlecock
grasping is a qualitative generalization demo, not a success-rate or general
grasping claim.

## 1. Match the reference hardware

- Robot: modified two-wheel XLeRobot, two SO-101-style arms, pan/tilt head,
  D455, right-wrist USB camera, LD06-style lidar, microphone and speaker.
- Robot computer: Ubuntu 24.04 x86_64 + ROS 2 Jazzy.
- GPU computer: NVIDIA Ubuntu/WSL2; verified on Ubuntu 24.04 WSL2 + RTX 3080.
- VLM: LM Studio serving `qwen/qwen3-vl-4b`.
- A single right Leader arm is needed **only for collecting your own data**.

See [BOM, wiring and mounting](docs/en/hardware.md). This is not a drop-in
configuration for every upstream XLeRobot.

## 2. Install on the two computers

On both computers, clone the same repository and edit the local templates:

```bash
git clone --recurse-submodules REPOSITORY_URL xlerobot_home_service_demo
cd xlerobot_home_service_demo
cp config/local.example.yaml config/local.yaml
cp .env.example .env
```

Device names, LAN URLs and file paths belong in `config/local.yaml`. Put the
same ACT token in both `.env` files. See the [installation guide](docs/en/install.md)
for prerequisites and the configuration checklist.

```bash
# GPU computer
./tools/setup gpu
# Use ./tools/setup gpu --with-gpd to include the native GPD backend.

# Robot computer
./tools/setup robot --with-person-detector --with-kws-model --generate-voice-prompts
```

The full voice/person-finding demo needs these explicit extras. The person
detector is AGPL-3.0; the KWS model terms are unresolved and it is downloaded
directly from its provider, not bundled here. See the installation guide and
[third-party notices](THIRD_PARTY_NOTICES.md).

**Model availability:** ACT weights and data are uploaded privately; public
release is pending. The [asset page](docs/en/assets.md) records availability.
Before publication, the demo needs the verified local checkpoint and its
manifest. You do not need to collect data or train a model to use the released
demo checkpoint.

## 3. Check dependencies

```bash
./tools/doctor gpu
./tools/doctor robot
```

Doctor is read-only and does not open cameras or motors. On a first setup,
missing calibration, map and unstarted services are expected unfinished steps;
complete them below and repeat doctor before running the demo.

## 4. Calibrate, then prepare the site

Follow the [calibration guide](docs/en/calibration.md):

```text
servo → head-camera → right-handeye ─┐
base (independent measurements) ────┴→ grasp-alignment → activate → render
```

Then [build a map, save the table place, validate and activate the site](docs/en/mapping.md).
A calibrated robot still needs its own map and `table` location. Do not copy
another robot's calibration or home map.

## 5. Try each grasp backend

```bash
# GPU computer: start LM Studio first, then
./tools/run gpu

# Robot computer, terminal 1
./tools/run demo --hardware

# Robot computer, terminal 2: submit ONE of these per trial
./tools/run grasp act --hardware
./tools/run grasp centroid --hardware
./tools/run grasp gpd --hardware
```

`grasp` submits a **complete fetch-and-deliver task** with the selected backend;
it is not a standalone arm-only command. The target comes from `demo.object_id`.
See [backend behavior and comparisons](docs/en/grasping.md).

## 6. Run the complete voice + web demo

The running `tools/run demo --hardware` stack waits for a request; it does not
submit one automatically. Open `http://<robot-host>:8080` or say “小乐小乐”,
then request the object. Follow the [demo guide](docs/en/demo.md).

```text
localize → navigate/dock → perceive → grasp and verify
→ find person → approach → speak → hand over
```

## 7. Tools, troubleshooting and assets

Want to collect your own demonstrations? Follow the separate
[ACT collection → conversion → training guide](docs/en/act-workflow.md).
Normal recording is **Start → Home → End → next episode**; valid recordings
are kept by default, and End lets you continue teleoperating to put the object down.

| Entry point | Use |
| --- | --- |
| `tools/setup robot\|gpu` | Install from source |
| `tools/doctor robot\|gpu` | Check local dependencies and services |
| `tools/calibrate` | Capture, solve, activate, replay and roll back calibration |
| `tools/act` | Collect, convert, train, evaluate and download |
| `tools/run` | GPU services, mapping, backend-selected tasks or the demo |

[Troubleshooting](docs/en/troubleshooting.md) · [Models and data](docs/en/assets.md)
· [Source layout](docs/en/README.md#source-layout)

Local configuration, maps, calibration, recordings, weights and logs are not
committed. Runtime assets normally live under ignored `.xlerobot/`. There is no
simulation/mock framework, deployment bundle or systemd installation.

Project-owned code and documentation: [Apache-2.0](LICENSE). ACT weights:
Apache-2.0. The 30-demo dataset: CC BY 4.0. Third-party assets retain their own
terms; see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
