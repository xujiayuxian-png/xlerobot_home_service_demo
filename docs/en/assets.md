# Models and data

[Documentation](README.md) · [中文](../zh-CN/assets.md)

## Run the demo without training

The ACT checkpoint is separate from the source repository. You do not need
the 30-demo dataset to run inference.

| Asset | Source / acquisition |
| --- | --- |
| ACT checkpoint | `lissajous/xlerobot-act-local-grasp-v1`; public |
| 30 yellow-glue-stick demonstrations | `lissajous/xlerobot-glue-stick-grasp-30`; public, optional for inference |
| SAM 2 | Downloaded and checked by `tools/setup gpu` |
| Qwen VLM | Load the recorded model in LM Studio; see [installation](install.md) |
| Whisper | Downloaded and checked by `tools/setup robot` |
| KWS | Explicit `--with-kws-model` provider download; model terms unresolved |
| Person detector | Explicit `--with-person-detector` AGPL extra |
| Spoken prompts | Generated locally with `--generate-voice-prompts` |

Current revisions and SHA-256 records are in
[the model inventory](../../assets/models/manifest.yaml),
[ACT download manifest](../../assets/models/xlerobot-act-local-grasp-v1.manifest.json),
[voice manifest](../../assets/models/voice-runtime.manifest.json) and
[dataset inventory](../../assets/data/manifest.yaml).
A separate [dataset checksum list](../../assets/data/xlerobot-glue-stick-grasp-30.sha256.json)
identifies the exact converted training files.
Both project repositories are public; downloads do not require a Hub login.

On the GPU computer, after setup:

```bash
./tools/act download
./tools/doctor gpu
```

The download entry installs and hash-checks the checkpoint and manifest at
the configured `models.act_checkpoint` / `models.act_manifest` paths.
Doctor may still report LM Studio or inference services that are not running;
continue with [demo startup](demo.md).

To replace the model, [train and evaluate your own](act-workflow.md). Do not
substitute the short software-smoke-test checkpoint for the demonstrated model.

## Training data

After GPU setup, download the converted dataset at the pinned public revision:

```bash
.xlerobot/venvs/act/bin/hf download lissajous/xlerobot-glue-stick-grasp-30 --repo-type dataset --revision 31e398471db6815d7ea900c0954f4e541d343f6e --local-dir .xlerobot/datasets/xlerobot-glue-stick-grasp-30
```

The repository includes `checksums.json` for its data, metadata and video files.
The [dataset card](https://huggingface.co/datasets/lissajous/xlerobot-glue-stick-grasp-30)
and [model card](https://huggingface.co/lissajous/xlerobot-act-local-grasp-v1)
are also available on the Hub.

All 30 original yellow-glue-stick demonstrations are publicly available,
not the later tool-acceptance recordings. See the
[dataset card](../../assets/data/glue-stick-grasp-30.md). The published LeRobot
dataset is a training input; it is not automatically a raw collection workspace
with `raw/` and `reviews/`.

For your own recordings, follow [collect → convert → train](act-workflow.md).
For an already-converted LeRobot dataset, pass its root directly to
`tools/act train --dataset PATH`; do not run raw conversion on it again.

## Terms and limits

Project ACT weights: Apache-2.0. Project demonstration dataset: CC BY 4.0.
Other models retain provider terms; see [third-party notices](../../THIRD_PARTY_NOTICES.md).
Only the glue-stick data trained the checkpoint. Other-object demonstrations
are qualitative, with no published success-rate benchmark.
