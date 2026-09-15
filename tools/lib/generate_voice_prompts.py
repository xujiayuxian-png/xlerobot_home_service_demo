#!/usr/bin/env python3
"""Generate the local preset voice clips that are intentionally absent from Git."""

from __future__ import annotations

import argparse
import asyncio
from pathlib import Path
import subprocess
import wave

import edge_tts
import yaml


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--check', action='store_true', help='Validate files without generating audio')
    return parser.parse_args()


async def generate(config_path: Path, output: Path, check=False):
    data = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    params = data['speak_text_server']['ros__parameters']
    voice = str(params['voice_id'])
    texts = list(params['preset_texts'])
    files = list(params['preset_files'])
    if len(texts) != len(files):
        raise SystemExit('preset_texts and preset_files have different lengths')
    if not check:
        output.mkdir(parents=True, exist_ok=True)
    for text, filename in zip(texts, files):
        target = output / filename
        if target.is_file() and target.stat().st_size > 0:
            if target.suffix == '.wav':
                with wave.open(str(target), 'rb') as audio:
                    if (audio.getnchannels(), audio.getsampwidth(), audio.getframerate(),
                            audio.getcomptype()) != (2, 2, 48000, 'NONE') or not audio.getnframes():
                        raise SystemExit(f'invalid fixed PCM: {target}')
            print(f'voice prompt exists: {target.name}')
            continue
        if check:
            raise SystemExit(f'missing voice prompt: {target}')
        temporary = target.with_suffix('.tmp.mp3')
        print(f'generating voice prompt: {target.name}')
        try:
            await edge_tts.Communicate(str(text), voice).save(str(temporary))
            if target.suffix == '.wav':
                pcm = target.with_suffix('.tmp.wav')
                try:
                    subprocess.run(['ffmpeg', '-nostdin', '-loglevel', 'error', '-y',
                                    '-i', str(temporary), '-ar', '48000', '-ac', '2',
                                    '-c:a', 'pcm_s16le', str(pcm)], check=True)
                    pcm.replace(target)
                finally:
                    pcm.unlink(missing_ok=True)
            else:
                temporary.replace(target)
        finally:
            temporary.unlink(missing_ok=True)


def main():
    args = parse_args()
    asyncio.run(generate(args.config, args.output, args.check))


if __name__ == '__main__':
    main()
