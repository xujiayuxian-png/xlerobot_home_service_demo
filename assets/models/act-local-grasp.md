# xlerobot-act-local-grasp-v1 model card

## Intended use

This ACT checkpoint supplies the local-contact phase of the reference robot's
grasp. The robot first reaches a calibrated pregrasp; the GPU service proposes
action chunks, and the robot-side streaming executor validates them before the
normal controller path.

## Training data and limits

The model was trained only from 30 demonstrations of grasping one yellow glue
stick. A shuttlecock, and every other object shown by the demo, is a qualitative
out-of-distribution generalization example. No success rate, broad object
coverage, or general grasp capability is claimed.

The planned public dataset is
`xujiayuxian-png/xlerobot-glue-stick-grasp-30`, all 30 demonstrations under
CC BY 4.0. Its large episode files are not stored in this Git repository and
the Hub upload is still pending. See `../data/glue-stick-grasp-30.md`.

## Model contract

- Policy: ACT with VAE enabled and automatic mixed precision disabled.
- Observation image: wrist RGB, `3 x 480 x 640`.
- Measured state and action: six values in the documented right-arm joint order.
- Chunk size and action steps: 100.
- Transformer: model dimension 512, feed-forward dimension 3200, four encoder
  layers, one decoder layer, eight attention heads, latent dimension 32.
- Visual backbone: ResNet-18 with ImageNet V1 weights.

## Verified runtime

- Ubuntu 24.04 in WSL2; Python 3.12.13.
- NVIDIA RTX 3080; CUDA 12.8; cuDNN 91002.
- LeRobot 0.5.1; PyTorch 2.10.0; torchvision 0.25.0.
- NumPy 2.2.6; safetensors 0.8.0; datasets 4.8.5;
  accelerate 1.14.0; huggingface-hub 1.20.1.

## Availability and license

The model is designated Apache-2.0 and its repository ID is fixed as
`xujiayuxian-png/xlerobot-act-local-grasp-v1`. The initial Hub upload has not
happened yet, so the manifest revision and download URL remain pending. After
publication, `tools/act download` will fetch only the immutable revision and
verify every file hash in `xlerobot-act-local-grasp-v1.manifest.json`.

`tools/act evaluate` is an offline deployment-qualification check. For a local
training output it records the fixed ACT structure, training provenance, and a
SHA-256 for every checkpoint file in `xlerobot_act_qualification/v1`. Configure
both that JSON and its checkpoint directory before serving. It is not a
real-object benchmark and must not be reported as a grasp success metric.
