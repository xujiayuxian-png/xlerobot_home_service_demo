# XLeRobot Home Service Demo

[中文说明](README.zh-CN.md)

This repository is the reproducible engineering record for one modified,
two-wheel XLeRobot that can hear a request, navigate to a table, grasp a
shuttlecock, find the nearest person, and deliver it. It is intentionally not a
general robot framework or a course.

**Watch the complete robot demo:**
[XLeRobot fetch-and-deliver on Bilibili](https://www.bilibili.com/video/BV1srNg6XEZj).

The two things worth copying are the unit-calibration workflow and the two
grasp routes:

```text
object request
├── classical: VLM/SAM2 + RGB-D geometry ── centroid or optional GPD target
└── hybrid ACT: calibrated MoveIt pregrasp ── wrist-image ACT action chunks
                                      │
                         robot-local validated execution
                                      │
                            grasp check and delivery
```

The default demo is `act` + `羽毛球`, with voice and the web console enabled.
The ACT model was trained on only 30 yellow-glue-stick demonstrations, so the
shuttlecock run is a qualitative out-of-distribution example, not a general
grasp claim. `centroid` and `gpd` select the classical route for comparison.

## 1. Reference hardware

- Robot computer: Ubuntu 24.04 x86_64, ROS 2 Jazzy.
- Modified differential-drive XLeRobot with two Feetech wheel servos, dual
  SO-101-style arms, pan/tilt head, 2D lidar, head-mounted RealSense D455, and a
  right-wrist USB camera.
- GPU computer: Ubuntu 24.04 under WSL2, tested with an RTX 3080.
- LM Studio serves `qwen/qwen3-vl-4b`; this repository runs classical proposal
  service port `8765` and ACT service port `8766`.

See [the hardware notes](docs/en/hardware.md) before assuming an upstream
XLeRobot has the same wiring, frames, servo IDs, or camera mounts.

## 2. Install on the Robot and GPU computers

Clone the same source on the GPU and robot computers:

```bash
git clone --recurse-submodules <repository-url>
cd xlerobot_home_service_demo
cp config/local.example.yaml config/local.yaml
cp .env.example .env
```

Edit both local files. Use the same ACT token on both computers, and put the
GPU computer's LAN URLs in `config/local.yaml`.

On the GPU computer, start LM Studio first, then install the two pinned Python
environments:

```bash
./tools/setup gpu
# Add --with-gpd if you want the optional native GPD candidate generator.
```

Before the initial Hub release, copy the vetted local checkpoint together with
its generated `model-manifest.json` qualification record to the two paths
configured as `models.act_checkpoint` and `models.act_manifest`. After an
immutable Hub revision is published, `tools/act download` performs this step.
The doctor deliberately fails until one of those two verifiable sources is in
place.

On the Robot computer, install ROS dependencies, the web console, voice
runtime, the optional AGPL person detector, and local voice prompts:

```bash
./tools/setup robot --with-person-detector --with-kws-model --generate-voice-prompts
```

Default voice needs `--with-kws-model`. That flag downloads directly from the
upstream provider after warning that the KWS weight/word-list terms are still
unclear; this repository neither bundles nor mirrors those files. See the
[complete installation notes](docs/en/install.md).

## 3. Check both computers

Run the read-only doctor before calibration or a demo:

```bash
./tools/doctor gpu
./tools/doctor robot
```

It checks exact environments, model hashes, service health, local artifacts,
and stable device names. It does not open a camera or motor device. Every
`ERROR` includes the failing boundary; common repairs are listed in
[troubleshooting](docs/en/troubleshooting.md).

## 4. Calibrate this unit

Do not copy another robot's numbers. The public workflow builds one immutable
bundle in this order:

```text
servo -> head-camera -> right-handeye
base -------------------------------> grasp-alignment -> activate -> render
```

Start with `./tools/calibrate status`. Live collection is available as
`./tools/calibrate capture <workflow> --hardware`; the two included camera
replays run without hardware. After all five components pass:

```bash
./tools/calibrate activate
./tools/calibrate render
./tools/calibrate status
```

The demo accepts only the checksum-valid rendered active bundle. Follow the
[calibration guide](docs/en/calibration.md) for targets, capture commands,
quality gates, resume, and rollback.

## 5. Run centroid, GPD, and ACT explicitly

Start the proposal services on the GPU and the calibrated Robot stack once:

```bash
# GPU computer
./tools/run gpu

# Robot computer, terminal 1
./tools/run demo --hardware
```

Then submit exactly one backend per request from a second Robot terminal:

```bash
./tools/run grasp centroid --hardware
./tools/run grasp gpd --hardware
./tools/run grasp act --hardware
```

These commands run the same fetch-and-deliver task while fixing its grasp
backend. GPD failure is reported as GPD failure; it never silently becomes a
centroid result. The traditional route is intentionally limited to this
top-grasp setup. See [the two-route guide](docs/en/grasping.md).

## 6. Run the complete voice and web demo

`./tools/run demo --hardware` starts the eight-stage Robot stack but does not
submit a task by itself:

```text
localize -> navigate/dock -> perceive object -> grasp and verify
-> find person -> approach -> speak -> hand over
```

Use the microphone or open `http://<robot-host>:8080`. Both interfaces default
to `act` and `羽毛球`; the web UI also exposes `centroid` and `gpd`. Every task
record stores both the requested and actual backend. See the
[full-demo guide](docs/en/demo.md).

## 7. Troubleshooting, assets, and repository boundary

Included: ROS source, configuration templates, calibration solvers and sample
replays, both grasp implementations, ACT HTTP service, and five top-level
commands (`setup`, `doctor`, `calibrate`, `act`, `run`). There is no simulation
or mock framework.

Not included in Git: a map of your home, unit calibration, recordings, model
weights, credentials, or the historical prerecorded MP3 clips. Model IDs,
known hashes, and availability are recorded in
[the asset manifest](assets/models/manifest.yaml). The ACT model is designated
Apache-2.0 and the 30-demo dataset CC BY 4.0; their fixed Hub repository IDs are
published as metadata, but both initial uploads are still pending.

The ACT data-to-model workflow is intentionally one entry point:

```bash
./tools/act collect --hardware
./tools/act convert --dry-run
./tools/act train --dry-run
./tools/act evaluate --checkpoint PATH --output PATH/model-manifest.json
```

The fifth operation, `tools/act download`, becomes usable only after the public
manifest records the immutable initial Hub revision. See the
[ACT workflow](docs/en/act-workflow.md).

The two-stage ACT behavior is also shown in the
[ACT route video](https://www.bilibili.com/video/BV18RK66JEdP).

Project-owned source and documentation are Apache-2.0. Third-party code,
models, and generated assets keep their own terms; see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and
[model notes](assets/models/external-models.md).
