"""The single supported X1 local demo profile; no remote fallback."""
import argparse
from pathlib import Path
import yaml

MODEL = 'qwen2.5-vl-3b-instruct-672x672-qnn2.36-w4a16-qcs8550'
URLS = {'lm_studio_url': 'http://127.0.0.1:18888',
        'act_url': 'http://127.0.0.1:18901',
        'npu_person_url': 'http://127.0.0.1:18902',
        'npu_asr_url': 'http://127.0.0.1:18903'}


def validate_profile(config):
    demo = config.get('demo', {})
    for key in ('inference_backend', 'asr_backend', 'person_backend', 'vlm_backend'):
        if demo.get(key) != 'npu':
            raise ValueError(f'demo.{key} must be npu for the local demo')
    if demo.get('grasp_backend') != 'act' or demo.get('act_wrist_only') is not True:
        raise ValueError('local demo requires wrist-only ACT')
    if config.get('models', {}).get('vlm') != MODEL:
        raise ValueError('local demo requires the verified 3B NPU model')
    for key, expected in URLS.items():
        if config.get('services', {}).get(key) != expected:
            raise ValueError(f'services.{key} must be {expected}; remote fallback is disabled')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--health', action='store_true')
    args = parser.parse_args()
    validate_profile(yaml.safe_load(Path(args.config).read_text()))
    if args.health:
        import json
        from urllib.request import urlopen
        for kind, key in [('act', 'act_url'), ('person', 'npu_person_url'), ('whisper', 'npu_asr_url')]:
            with urlopen(URLS[key]+'/healthz', timeout=3) as response:
                value = json.load(response)
            if value.get('kind') != kind or value.get('device') != 'qnn-htp' or value.get('cpu_fallback') is not False:
                raise ValueError(f'{kind} is not the expected NPU worker')
        with urlopen(URLS['lm_studio_url']+'/v1/models', timeout=3) as response:
            models = json.load(response)['data']
        if not any(model.get('id') == MODEL for model in models):
            raise ValueError('NPU VLM is not ready')
    print('X1 local profile: all inference endpoints are loopback; wrist-only ACT')
