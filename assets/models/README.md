# Model assets

This directory contains metadata only. No model weight is part of the source
repository or covered automatically by its Apache-2.0 license.

Before using a model, obtain it from its documented provider, review its
license for your use, and verify the checksum when one is recorded in
`manifest.yaml`. Put local files under ignored, repo-relative `.xlerobot/`
paths referenced from `config/local.yaml`.

The default demo needs three independently supplied model runtimes:

1. LM Studio serving `qwen/qwen3-vl-4b` through an OpenAI-compatible endpoint.
2. The six-action, wrist-camera ACT checkpoint described in
   `act-local-grasp.md`.
3. A person detector configured as `models.person_detector`. The reference
   Ultralytics dependency is an explicitly selected AGPL-3.0 extra.

The ACT checksum set identifies the model tested by the original project. The
model is designated Apache-2.0, but its first Hub upload and immutable revision
are still pending.

Robot setup downloads the named `Systran/faster-whisper-small` snapshot. Voice
mode also needs the sherpa-onnx KWS artifact, but its model/word-list terms are
unresolved; only the explicit `--with-kws-model` flag downloads it directly
from the recorded provider. Classical segmentation uses the pinned SAM 2
source revision and model snapshot in the manifest. See `external-models.md`
for the separation between source, model, and generated audio licenses.
