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
- Imported source revision: `51ca0ec31bdb48713b94bacdba828bf8d889296b`
- Current tracked mesh inventory digest (`find ... -print0 | sort -z |`
  `xargs -0 sha256sum | sha256sum`, run from the repository root):
  `bf4fc39f0fd019ca2df367016a44b1965a4f5bf0ae6066ee09ee8bc0eaeafb29`

Project-specific wheel, sensor, arm-mount, and camera calibration values are
documented separately and are not presented as upstream defaults.

## SCServo Linux SDK

`third_party/SCServo_Linux` is a pinned Git submodule. Its source and license
remain in that submodule; `xlerobot_feetech` provides the project-owned ROS 2
boundary around it.

- Source: <https://github.com/adityakamath/SCServo_Linux>
- Commit: `da2ed3acab00a9da69dedff914b4c69e89e93aa8`
- License: MIT (`third_party/SCServo_Linux/LICENSE`)
- License-file SHA-256:
  `45baea5cca9a0bb0b470609d459f6e17ba012c2b87ed472935e58581296a2c7f`

## GPD

The optional native grasp candidate generator is fetched during explicit GPU
setup; its source is not vendored in this repository.

- Source: <https://github.com/atenpas/gpd>
- Commit: `6327f20eabfcba41a05fdd2e2ba408153dc2e958`
- Upstream license: BSD-2-Clause
- Project patch hashes:
  `22ecebcf00f0f3769b9dbbf5124c07366010c7e86a3e6bb64ecf9b6447ddda7e`
  (`detect_grasps_json.cpp`) and
  `4b813cc3a9705a6e3fb0e6f1cb6bd63f754557e0f94cd774bbb0380cb114686d`
  (`gpd-cmake.patch`)

The built binary remains subject to the upstream BSD-2-Clause terms. Review
the upstream license in the pinned checkout before redistributing a binary.

## SAM 2

- Source: <https://github.com/facebookresearch/sam2>
- Commit: `2b90b9f5ceec907a1c18123530e92e794ad901a4`
- Source license: Apache-2.0
- Model: `facebook/sam2.1-hiera-tiny`
- Model snapshot: `de431c4043854a71d8101e17995dfe596bf101a5`
- Checkpoint SHA-256:
  `7402e0d864fa82708a20fbd15bc84245c2f26dff0eb43a4b5b93452deb34be69`
- Verified Hub client: `huggingface-hub==0.36.2`
- Classical requirements lock SHA-256:
  `617c37fc9cb7820d1c8ed0dfb57d412b6c2d6d61d22389e5eaf420701025cd57`

The model card records Apache-2.0 for the checkpoint. It is downloaded from
its provider and is not redistributed in Git; preserve the provider metadata
when publishing a cache or derived bundle.

## LeRobot and ACT runtime

- Source: <https://github.com/huggingface/lerobot>
- Package version: `0.5.1`
- Source license: Apache-2.0 (with the upstream bundled notices)
- ACT requirements lock SHA-256:
  `2fa6a5ae09036ea3200be998ce32b9ade64690970cf07b6054c96ed89cf1e479`

The project-owned model `xujiayuxian-png/xlerobot-act-local-grasp-v1` is a
separate Apache-2.0 artifact; its model card and exact file hashes live under
`assets/models/`. The CC BY 4.0 training dataset is also a separate release.

## Qwen through LM Studio

The verified GPU used LM Studio `0.4.12+1`, local model revision `4`, and the
served identifier `qwen/qwen3-vl-4b`. The concrete Apache-2.0 artifact is
`lmstudio-community/Qwen3-VL-4B-Instruct-GGUF` at revision
`9eaf9988fe9b5e33541dc614622c36d5e90dd509`:

- `Qwen3-VL-4B-Instruct-Q4_K_M.gguf`: 2,497,281,568 bytes, SHA-256
  `aa18af1fdda59081005319536eebff9406d556ce8a753d9a3aa81cf635eadde7`.
- `mmproj-Qwen3-VL-4B-Instruct-F16.gguf`: 836,180,160 bytes, SHA-256
  `1b70e983d69dd424eb16e0ed2104d0b2a58bf01d866657f4de5da6c963620f19`.
- LM model-descriptor archive: SHA-256
  `e8869e92a4e59986e9aeac41fb91491d4267ab7b13aa0dbe3d7fd7c6da8e3db3`.

Neither LM Studio nor these large files are included in Git. The doctor can
confirm the served API identifier; the concrete local artifact identity must
be checked in LM Studio against the recorded revision and hashes.

## Local voice models

Robot setup can download, but this Git repository does not redistribute, the
following voice artifacts:

- KWS release asset:
  `sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20`; runtime
  `sherpa-onnx==1.13.4` and `onnxruntime==1.27.0`.
- Speech recognition snapshot: `Systran/faster-whisper-small` at immutable
  revision `536b0662742c02347bc0e980a01041f333bce120`; runtime
  `faster-whisper==1.2.1` and `ctranslate2==4.8.1`.

Exact runtime-file hashes are in `assets/models/voice-runtime.manifest.json`.
The faster-whisper model card records MIT. The KWS release asset does not have
a sufficiently clear model/word-list license in the reviewed source; upstream
[issue #3802](https://github.com/k2-fsa/sherpa-onnx/issues/3802) remains open.
Its terms therefore remain unresolved: the repository neither redistributes
nor mirrors it, and setup fetches it from the provider only after the user
passes `--with-kws-model` explicitly.

## Web console dependencies

The React/Vite console uses exact npm versions and per-package license and
integrity metadata in `ros2_ws/src/xlerobot_hmi/web/package-lock.json`.

- Direct versions and recorded licenses: React/React DOM `19.0.0` (MIT), Vite
  `6.4.3` (MIT), TypeScript `5.7.3` (Apache-2.0), `@vitejs/plugin-react`
  `4.3.4` (MIT), Vitest `3.2.7` (MIT), jsdom `26.0.0` (MIT), and
  `@testing-library/react` `16.2.0` (MIT).
- Lockfile SHA-256:
  `b39645e591ad6d2fea7322a15f40e3aa8636d021b59f48b595390f2b15470cff`

The lockfile is the hash source of record for transitive npm archives. Review
all recorded package licenses before shipping a prebuilt frontend bundle.

## Optional person detector and generated speech

Ultralytics `8.4.58` and the reference `yolov8n.pt` are an explicit optional
AGPL-3.0 extra and are not bundled. The weight is fetched from the Ultralytics
v8.3.0 release assets and checked as 6,549,796 bytes with SHA-256
`f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36`.
The historical Edge TTS MP3 clips are also
not bundled because their history does not establish redistribution rights;
fresh clips are generated locally and remain ignored.
