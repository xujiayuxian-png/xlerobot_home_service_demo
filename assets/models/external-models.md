# External model and generated-asset notes

The repository's Apache-2.0 license applies to project-owned source and docs,
not automatically to externally downloaded code or weights. Each user is
responsible for reviewing the provider's current terms before downloading or
redistributing an asset.

- LM Studio supplies an OpenAI-compatible endpoint for
  `qwen/qwen3-vl-4b`; neither the server nor model is vendored here. The exact
  LM Studio version and concrete Q4_K_M GGUF revision, sizes, and hashes are in
  `manifest.yaml` and `THIRD_PARTY_NOTICES.md`.
- SAM 2 code and the Hiera Tiny checkpoint are fetched at the immutable
  revisions in `manifest.yaml`.
- The project-owned ACT checkpoint is Apache-2.0 and uploaded privately,
  awaiting public release; the 30-demonstration training dataset is a separate CC BY 4.0
  release described under `../data/`.
- The local voice path uses the exact sherpa-onnx release asset and immutable
  faster-whisper snapshot recorded in `voice-runtime.manifest.json`. Robot
  setup stores Whisper under an ignored `.xlerobot/models/` path. Because the
  KWS model/word-list terms remain unresolved, only `--with-kws-model` fetches
  that artifact directly from its provider. Every selected runtime file is
  hash-checked before use.
- The nearest-person implementation is optional because the reference
  Ultralytics package and weights are distributed under AGPL-3.0. Install it
  explicitly with `tools/setup robot --with-person-detector`.
- The 22 historical Chinese MP3 clips are absent: project history says they
  were produced with Edge TTS, but contains neither a source-generation record
  nor evidence granting redistribution. A user may generate fresh local clips
  with `tools/setup robot --generate-voice-prompts`; generated files stay
  ignored and are not relicensed by this repository.
