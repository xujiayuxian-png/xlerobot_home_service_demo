#!/usr/bin/env python3
"""Generate the local preset voice clips that are intentionally absent from Git."""

from __future__ import annotations

import argparse
import asyncio
from pathlib import Path

import edge_tts
import yaml


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    return parser.parse_args()


async def generate(config_path: Path, output: Path):
    data = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    params = data['speak_text_server']['ros__parameters']
    voice = str(params['voice_id'])
    texts = list(params['preset_texts'])
    files = list(params['preset_files'])
    if len(texts) != len(files):
        raise SystemExit('preset_texts and preset_files have different lengths')
    output.mkdir(parents=True, exist_ok=True)
    for text, filename in zip(texts, files):
        target = output / filename
        if target.is_file() and target.stat().st_size > 0:
            print(f'voice prompt exists: {target.name}')
            continue
        temporary = target.with_suffix('.tmp.mp3')
        print(f'generating voice prompt: {target.name}')
        await edge_tts.Communicate(str(text), voice).save(str(temporary))
        temporary.replace(target)


def main():
    args = parse_args()
    asyncio.run(generate(args.config, args.output))


if __name__ == '__main__':
    main()
