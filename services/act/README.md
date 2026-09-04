# ACT inference service

This GPU-side service loads a local LeRobot 0.5.1 ACT checkpoint and exposes:

- `GET /healthz`
- authenticated `POST /predict`
- authenticated `POST /reset`

Requests contain a six-value measured joint state plus the image features
declared by the checkpoint (`head`, `wrist`, or both). Responses contain a
bounded six-value action chunk. The process has no ROS dependency and never
opens a robot device.

Use the repository entry points instead of invoking modules directly:

```bash
./tools/act collect --hardware
./tools/act convert --dry-run
./tools/act train --dry-run
./tools/act evaluate --checkpoint PATH --output PATH/model-manifest.json
./tools/run gpu
```

The checkpoint is not stored in Git. Configure its repo-relative ignored path
and its generated or downloaded manifest in `config/local.yaml`, set
`XLEROBOT_ACT_TOKEN` in `.env`, and read
`assets/models/act-local-grasp.md` before running the service.

`evaluate` hashes every file and records local-training provenance plus the
validated ACT structure. Once the initial immutable Hub revision exists,
`tools/act download` installs the fixed public manifest instead. Both schemas
are checked before the service loads a checkpoint.

The recorded reference environment is Ubuntu 24.04 under WSL2, Python 3.12.13,
an RTX 3080, CUDA 12.8, cuDNN 91002, PyTorch 2.10.0, and torchvision 0.25.0.
The key Python versions are pinned in `requirements.lock.txt`; after setup, the
complete environment resolved on your host is captured under the ignored
`.xlerobot/environment/` directory.
