"""Export the reference wrist-only ACT checkpoint for offline inference; no robot I/O."""
import argparse
import hashlib
import json
from pathlib import Path


def validate_config(config):
    if (config['input_features'] != {
            'observation.state': {'type': 'STATE', 'shape': [6]},
            'observation.images.wrist': {'type': 'VISUAL', 'shape': [3, 480, 640]}}
            or config['n_obs_steps'] != 1 or config['chunk_size'] != 100 or config['dim_model'] != 512
            or config['latent_dim'] != 32 or config['vision_backbone'] != 'resnet18'
            or config.get('replace_final_stride_with_dilation')
            or config['output_features']['action']['shape'] != [6]):
        raise ValueError('exporter requires the reference wrist-only 480x640, 6-joint, 100-step ACT')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--samples', type=Path, nargs='*', default=[])
    args = parser.parse_args()
    config_json = json.loads((args.checkpoint / 'config.json').read_text())
    validate_config(config_json)
    import numpy as np
    import onnx
    import onnxruntime as ort
    import torch
    from safetensors.torch import load_file
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors
    torch.set_num_threads(2)
    torch.manual_seed(17)
    cfg = PreTrainedConfig.from_pretrained(args.checkpoint, local_files_only=True)
    cfg.pretrained_backbone_weights = None  # Checkpoint supplies all weights; do not download ImageNet.
    cfg.device = 'cpu'
    policy = ACTPolicy.from_pretrained(args.checkpoint, config=cfg, local_files_only=True).cpu().eval()
    preprocess, postprocess = make_pre_post_processors(
        policy.config, pretrained_path=str(args.checkpoint),
        preprocessor_overrides={'device_processor': {'device': 'cpu'}},
        postprocessor_overrides={'device_processor': {'device': 'cpu'}})
    pre = load_file(str(args.checkpoint / 'policy_preprocessor_step_3_normalizer_processor.safetensors'))
    post = load_file(str(args.checkpoint / 'policy_postprocessor_step_0_unnormalizer_processor.safetensors'))

    class ExportModel(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.core = policy.model
            for prefix, key in [('image', 'observation.images.wrist'), ('state', 'observation.state')]:
                self.register_buffer(prefix + '_mean', pre[key + '.mean'])
                self.register_buffer(prefix + '_std', pre[key + '.std'] + 1e-8)
            self.register_buffer('action_mean', post['action.mean'])
            self.register_buffer('action_std', post['action.std'])
            with torch.no_grad():
                pos = self.core.encoder_cam_feat_pos_embed(torch.zeros(1, 512, 15, 20))
                pos = torch.cat((self.core.encoder_1d_feature_pos_embed.weight.unsqueeze(1),
                                 pos.flatten(2).permute(2, 0, 1)), dim=0)
                self.register_buffer('position', pos)
                self.register_buffer('latent', self.core.encoder_latent_input_proj(torch.zeros(1, 32)).unsqueeze(0))
                self.register_buffer('queries', torch.zeros(100, 1, 512))

        def forward(self, image, state):
            image = (image-self.image_mean)/self.image_std
            state = (state-self.state_mean)/self.state_std
            features = self.core.backbone(image)['feature_map']
            features = self.core.encoder_img_feat_input_proj(features).flatten(2).permute(2, 0, 1)
            tokens = torch.cat((self.latent, self.core.encoder_robot_state_input_proj(state).unsqueeze(0),
                                features), dim=0)
            memory = self.core.encoder(tokens, pos_embed=self.position)
            output = self.core.decoder(self.queries, memory, encoder_pos_embed=self.position,
                                       decoder_pos_embed=self.core.decoder_pos_embed.weight.unsqueeze(1))
            return self.core.action_head(output.transpose(0, 1))*self.action_std + self.action_mean

    model = ExportModel().eval()
    samples = []
    for index in range(3):
        image = torch.rand(1, 3, 480, 640) if index else torch.zeros(1, 3, 480, 640)
        state = pre['observation.state.mean'][None] + (index-1)*0.25*pre['observation.state.std'][None]
        samples.append((image, state))
    for path in args.samples:
        with np.load(path, allow_pickle=False) as sample:
            image, state = sample['image'], sample['state']
        if (image.shape != (1, 3, 480, 640) or state.shape != (1, 6)
                or image.dtype != np.float32 or state.dtype != np.float32
                or not np.isfinite(image).all() or not np.isfinite(state).all()):
            raise ValueError(f'invalid saved observation: {path}')
        samples.append((torch.from_numpy(image), torch.from_numpy(state)))
    reference_errors = []
    with torch.inference_mode():
        for image, state in samples:
            processed = preprocess({'observation.images.wrist': image[0], 'observation.state': state[0]})
            reference = postprocess(policy.predict_action_chunk(processed))
            output = model(image, state)
            reference_errors.append(float((reference-output).abs().max()))
            torch.testing.assert_close(output, reference, atol=1e-5, rtol=1e-5)
    args.output.mkdir(parents=True, exist_ok=True)
    path = args.output / 'act-float.onnx'
    with torch.no_grad():
        torch.onnx.export(model, samples[0], str(path), input_names=['image', 'state'],
                          output_names=['actions'], opset_version=17, dynamo=False,
                          do_constant_folding=True)
    graph = onnx.load(str(path))
    onnx.checker.check_model(graph)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 2
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(path), options, providers=['CPUExecutionProvider'])
    errors = []
    with torch.inference_mode():
        for index, (image, state) in enumerate(samples):
            reference = model(image, state).numpy()
            output = session.run(None, {'image': image.numpy(), 'state': state.numpy()})[0]
            np.testing.assert_allclose(output, reference, atol=1e-4, rtol=1e-4)
            errors.append(float(np.max(np.abs(output-reference))))
            np.savez(args.output/f'sample-{index}.npz', image=image.numpy(), state=state.numpy(), reference=reference)
    report = {'scope': '3 synthetic inputs plus saved observations; no robot execution',
              'saved_observations': [str(p) for p in args.samples], 'shape': [1, 100, 6],
              'wrapper_reference_max_abs': reference_errors, 'onnx_max_abs': errors,
              'onnx_nodes': len(graph.graph.node), 'operators': sorted({n.op_type for n in graph.graph.node}),
              'checkpoint_sha256': hashlib.sha256((args.checkpoint/'model.safetensors').read_bytes()).hexdigest()}
    (args.output/'export-report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
