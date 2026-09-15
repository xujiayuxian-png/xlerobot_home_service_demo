"""Offline X1 inference experiments. No ROS, microphone, camera or motor access."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import resource
import time

import numpy as np


def check(result):
    if result != 0:
        raise RuntimeError(f'AidLite operation failed: {result}')


def load_qnn(path, version, shapes=None):
    import aidlite
    model = aidlite.Model.create_instance(str(path))
    if model is None:
        raise RuntimeError('cannot create QNN model')
    if shapes:
        model.set_model_properties(*shapes)
    config = aidlite.Config.create_instance()
    config.framework_type = getattr(aidlite.FrameworkType, 'TYPE_QNN' + version)
    config.accelerate_type = aidlite.AccelerateType.TYPE_DSP
    config.is_quantify_model = 1
    interpreter = aidlite.InterpreterBuilder.build_interpreter_from_model_and_config(model, config)
    if interpreter is None:
        raise RuntimeError('cannot create QNN interpreter')
    try:
        check(interpreter.init())
        check(interpreter.load_model())
    except BaseException:
        interpreter.destroy()
        raise
    return interpreter


def evidence():
    libraries = sorted({line.split()[-1] for line in Path('/proc/self/maps').read_text().splitlines()
                        if any(name in line for name in ('Qnn', 'aidlite', 'aidgen', 'cdsprpc'))})
    devices = []
    for fd in Path('/proc/self/fd').iterdir():
        try:
            target = os.readlink(fd)
            if any(name in target for name in ('adsprpc', 'fastrpc', 'cdsp')):
                devices.append(target)
        except OSError:
            pass
    return {'libraries': libraries, 'rpc_devices': sorted(set(devices))}


def cpu_seconds():
    usage = resource.getrusage(resource.RUSAGE_SELF)
    return usage.ru_utime + usage.ru_stime


def summary(values):
    return {'mean': float(np.mean(values)), 'p95': float(np.percentile(values, 95)),
            'max': float(np.max(values))}


def person_boxes(scores, xywh, shape, scale, threshold=0.6):
    """Decode only COCO person; boxes use the vendor's top-left square padding."""
    scores = np.asarray(scores).reshape(80, 8400)[0]
    xywh = np.asarray(xywh).reshape(4, 8400).T
    selected = np.isfinite(scores) & (scores >= threshold) & np.isfinite(xywh).all(axis=1)
    scores, boxes = scores[selected], xywh[selected].copy()
    boxes[:, :2] -= boxes[:, 2:] / 2
    boxes[:, 2:] += boxes[:, :2]
    boxes *= scale
    boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, shape[1])
    boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, shape[0])
    valid = (boxes[:, 2] > boxes[:, 0]) & (boxes[:, 3] > boxes[:, 1])
    scores, boxes = scores[valid], boxes[valid]
    order = scores.argsort()[::-1]
    kept = []
    while order.size and len(kept) < 100:
        index = order[0]
        kept.append({'bbox_px': boxes[index].tolist(), 'confidence': float(scores[index])})
        rest = order[1:]
        intersection = np.maximum(0, np.minimum(boxes[index, 2:], boxes[rest, 2:]) -
                                  np.maximum(boxes[index, :2], boxes[rest, :2])).prod(axis=1)
        areas = (boxes[:, 2:] - boxes[:, :2]).prod(axis=1)
        iou = intersection / np.maximum(areas[index] + areas[rest] - intersection, 1e-12)
        order = rest[iou <= 0.45]
    return kept


def run_yolo(args):
    import cv2
    cv2.setNumThreads(1)
    image = cv2.imread(str(args.image))
    if image is None:
        raise ValueError('input image could not be decoded')
    interpreter = None
    loaded = time.perf_counter()
    if args.backend == 'qnn236':
        import aidlite
        if args.size != 640:
            raise ValueError('this QNN artifact has fixed 640x640 input')
        interpreter = load_qnn(args.model, '236', (
            [[1, 640, 640, 3]], aidlite.DataType.TYPE_FLOAT32,
            [[1, 80, 8400], [1, 4, 8400]], aidlite.DataType.TYPE_FLOAT32))
        def detect(frame):
            start = time.perf_counter()
            length = max(frame.shape[:2])
            square = np.zeros((length, length, 3), np.uint8)
            square[:frame.shape[0], :frame.shape[1]] = frame
            rgb = cv2.resize(cv2.cvtColor(square, cv2.COLOR_BGR2RGB), (640, 640))
            tensor = (rgb.astype(np.float32) / 255)[None]
            check(interpreter.set_input_tensor(0, tensor))
            before_invoke = time.perf_counter()
            check(interpreter.invoke())
            after_invoke = time.perf_counter()
            boxes = person_boxes(interpreter.get_output_tensor(0), interpreter.get_output_tensor(1),
                                 frame.shape, length / 640)
            return boxes, (after_invoke - before_invoke) * 1000, (time.perf_counter()-start)*1000
    else:
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(2)
        model = YOLO(str(args.model))
        def detect(frame):
            start = time.perf_counter()
            pred = model.predict(frame, classes=[0], conf=0.6, imgsz=args.size,
                                 device='cpu', verbose=False)[0]
            boxes = [{'bbox_px': row.xyxy[0].tolist(), 'confidence': float(row.conf[0])}
                     for row in pred.boxes]
            elapsed = (time.perf_counter()-start)*1000
            return boxes, float(pred.speed['inference']), elapsed
    try:
        load_s = time.perf_counter() - loaded
        first = detect(image)
        negative = detect(np.zeros_like(image))[0]
        runtime = evidence()
        if interpreter and not any('libQnnHtp.so' in p for p in runtime['libraries']):
            raise RuntimeError('requested DSP backend but no QNN HTP library was observed')
        started, cpu_start = time.perf_counter(), cpu_seconds()
        records = []
        while time.perf_counter() - started < args.duration:
            boxes, invoke, total = detect(image)
            records.append({'invoke_ms': invoke, 'end_to_end_ms': total})
            time.sleep(max(0, started + len(records) / args.rate - time.perf_counter()))
        elapsed, cpu = time.perf_counter()-started, cpu_seconds()-cpu_start
        return {'kind': 'person_detection', 'backend': args.backend, 'input_size': args.size,
                'load_s': load_s, 'first_frame_ms': first[2], 'detections': first[0],
                'black_image_detections': negative, 'runtime': runtime,
                'requested_rate_hz': args.rate, 'actual_rate_hz': len(records)/elapsed,
                'duration_s': elapsed, 'cpu_s': cpu, 'cpu_one_core_percent': 100*cpu/elapsed,
                'invoke_ms': summary([r['invoke_ms'] for r in records]),
                'end_to_end_ms': summary([r['end_to_end_ms'] for r in records]), 'samples': records}
    finally:
        if interpreter:
            interpreter.destroy()


def decoder_controls(token, position, context=200):
    if not 0 <= position < context:
        raise ValueError('decoder position outside context')
    mask = np.full((1, 1, 1, context), -100.0, np.float32)
    mask[..., -(position+1):] = 0
    # AidLite's non-native API converts FLOAT32 to the model's INT32 tensors.
    # Supplying an int32 buffer here silently corrupts IDs, repeating language tokens.
    return np.array([[token]], np.float32), np.array([position], np.float32), mask


def token_log_probability(logits, token):
    values = np.asarray(logits, dtype=np.float64).reshape(-1)
    if not np.isfinite(values).all() or not 0 <= token < len(values):
        raise ValueError('invalid decoder logits or token')
    shifted = values - np.max(values)
    return float(shifted[token] - np.log(np.exp(shifted).sum()))


def asr_candidate_passes(result):
    """Offline screening only; does not replace the live voice quality gate."""
    silence = result.get('no_speech_probability')
    confidence = result.get('mean_token_log_probability')
    return bool(result.get('finished') and result.get('text', '').strip()
                and silence is not None and np.isfinite(silence) and 0 <= silence <= 0.6
                and confidence is not None and np.isfinite(confidence) and confidence >= -0.8)


class QnnWhisper:
    def __init__(self, directory):
        from tokenizers import Tokenizer
        import re
        self.encoder = load_qnn(directory / 'encoder_model_htp.bin.aidem', '240')
        try:
            self.decoder = load_qnn(directory / 'decoder_model_htp.bin.aidem', '240')
            self.tokenizer = Tokenizer.from_file(str(directory / 'tokenizer.json'))
            self.inputs = {x.name: x for x in self.decoder.get_input_tensor_info()[0]}
            self.prefix = [self.tokenizer.token_to_id(t) for t in
                           ('<|startoftranscript|>', '<|zh|>', '<|transcribe|>', '<|notimestamps|>')]
            self.language_ids = sorted(i for token, i in self.tokenizer.get_vocab().items()
                                       if re.fullmatch(r'<\|[a-z]{2,3}\|>', token))
            if any(t is None for t in self.prefix) or self.inputs['attention_mask'].shape != [1, 1, 1, 200]:
                raise ValueError('unexpected tokenizer or decoder context; this probe requires the verified artifact')
        except BaseException:
            if hasattr(self, 'decoder'):
                self.decoder.destroy()
            self.encoder.destroy()
            raise

    def transcribe(self, audio, max_tokens):
        from faster_whisper.feature_extractor import FeatureExtractor
        if not 0 < len(audio) <= 480000:
            raise ValueError('audio must contain 0..30 seconds at 16 kHz')
        features = FeatureExtractor()(np.pad(audio, (0, 480000-len(audio))), padding=0)[None]
        check(self.encoder.set_input_tensor('input_features', np.ascontiguousarray(features, dtype=np.float32)))
        begin = time.perf_counter()
        check(self.encoder.invoke())
        encoder_ms = 1000*(time.perf_counter()-begin)
        for tensor in self.encoder.get_output_tensor_info()[0]:
            check(self.decoder.set_input_tensor(tensor.name, self.encoder.get_output_tensor(tensor.name).copy()))
        for name, tensor in self.inputs.items():
            if '_self_' in name:
                check(self.decoder.set_input_tensor(name, np.zeros(tensor.shape, np.float32)))
        tokens = list(self.prefix)
        eos = self.tokenizer.token_to_id('<|endoftext|>')
        no_speech = self.tokenizer.token_to_id('<|nospeech|>')
        if no_speech is None:
            no_speech = self.tokenizer.token_to_id('<|nocaptions|>')
        no_speech_probability = None
        language_probability = None
        token_log_probabilities = []
        finished = False
        for position in range(max_tokens):
            token, index, mask = decoder_controls(tokens[position], position)
            for name, data in [('input_ids', token), ('position_ids', index), ('attention_mask', mask)]:
                check(self.decoder.set_input_tensor(name, data))
            check(self.decoder.invoke())
            if position == 0 and no_speech is not None:
                first_logits = self.decoder.get_output_tensor('logits').copy().reshape(-1)
                no_speech_probability = float(np.exp(token_log_probability(first_logits, no_speech)))
                language_probability = float(np.exp(token_log_probability(
                    first_logits[self.language_ids], self.language_ids.index(self.prefix[1]))))
            if position >= len(self.prefix)-1:
                logits = self.decoder.get_output_tensor('logits').copy().reshape(-1)
                raw_logits = logits.copy()
                logits[self.prefix[-1]+1:] = -np.inf  # No timestamp tokens for this short-command probe.
                token = int(logits.argmax())
                token_log_probabilities.append(token_log_probability(raw_logits, token))
                tokens.append(token)
                if token == eos:
                    finished = True
                    break
            for name in self.inputs:
                if '_self_' in name:
                    check(self.decoder.set_input_tensor(name, self.decoder.get_output_tensor(
                        name.replace('_in', '_out')).copy()))
        return {'text': self.tokenizer.decode(tokens), 'finished': finished,
                'encoder_ms': encoder_ms, 'decoder_steps': position+1, 'tokens': tokens,
                'no_speech_probability': no_speech_probability,
                'language_probability': language_probability,
                'mean_token_log_probability': float(np.mean(token_log_probabilities))}

    def close(self):
        self.decoder.destroy()
        self.encoder.destroy()


def run_whisper(args):
    from faster_whisper.audio import decode_audio
    started = time.perf_counter()
    if args.backend == 'cpu':
        from faster_whisper import WhisperModel
        model = WhisperModel(str(args.model), device='cpu', compute_type='int8',
                             cpu_threads=2, num_workers=1, local_files_only=True)
    else:
        model = QnnWhisper(args.model)
    try:
        report = {'kind': 'short_command_asr', 'backend': args.backend, 'load_s': time.perf_counter()-started,
                  'decoder': ('Chinese beam=3, int8, 2 CPU threads, no VAD' if args.backend == 'cpu'
                              else 'Chinese greedy, float cache transfer, no VAD'),
                  'runtime': evidence(), 'runs': []}
        for repeat in range(args.repeat):
            for path in args.audio:
                start, cpu_start = time.perf_counter(), cpu_seconds()
                audio = decode_audio(str(path), sampling_rate=16000)
                if args.backend == 'cpu':
                    segments, _ = model.transcribe(
                        audio, language='zh', beam_size=3, best_of=1, temperature=0.0,
                        condition_on_previous_text=False, without_timestamps=True,
                        vad_filter=False, hotwords='羽毛球',
                        initial_prompt='这是家庭服务机器人取物命令，常见物体名称包括羽毛球。')
                    segments = list(segments)
                    result = {'text': ''.join(s.text.strip() for s in segments),
                              'no_speech_probabilities': [s.no_speech_prob for s in segments]}
                else:
                    result = model.transcribe(audio, args.max_tokens)
                    result['candidate_quality_passed'] = asr_candidate_passes(result)
                elapsed, cpu = time.perf_counter()-start, cpu_seconds()-cpu_start
                result.update(audio=str(path), repeat=repeat+1, audio_s=len(audio)/16000,
                              elapsed_s=elapsed, cpu_s=cpu, cpu_one_core_percent=100*cpu/elapsed)
                report['runs'].append(result)
                print(json.dumps(result, ensure_ascii=False), flush=True)
        return report
    finally:
        if args.backend != 'cpu':
            model.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    yolo = commands.add_parser('yolo')
    yolo.add_argument('--image', type=Path, required=True)
    yolo.add_argument('--backend', choices=['qnn236', 'cpu'], default='qnn236')
    yolo.add_argument('--size', type=int, default=640)
    yolo.add_argument('--duration', type=int, default=60)
    yolo.add_argument('--rate', type=float, default=3)
    whisper = commands.add_parser('whisper')
    whisper.add_argument('--backend', choices=['qnn240', 'cpu'], default='qnn240')
    whisper.add_argument('--audio', nargs='+', type=Path, required=True)
    whisper.add_argument('--repeat', type=int, default=3)
    whisper.add_argument('--max-tokens', type=int, default=64)
    for command in (yolo, whisper):
        command.add_argument('--model', type=Path, required=True)
        command.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.model.exists():
        parser.error('model path does not exist')
    if args.command == 'yolo' and (not 1 <= args.duration <= 600 or not 0 < args.rate <= 30):
        parser.error('duration must be 1..600 seconds and rate (0, 30] Hz')
    if args.command == 'whisper' and (not 1 <= args.repeat <= 10 or not 4 <= args.max_tokens <= 199):
        parser.error('repeat must be 1..10 and max-tokens 4..199')
    report = run_yolo(args) if args.command == 'yolo' else run_whisper(args)
    report['max_rss_kib'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    report['online_cpus'] = os.cpu_count()
    report['model'] = str(args.model.resolve())
    if args.model.is_file():
        digest = hashlib.sha256()
        with args.model.open('rb') as model_file:
            for chunk in iter(lambda: model_file.read(1024 * 1024), b''):
                digest.update(chunk)
        report['model_sha256'] = digest.hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    args.output.chmod(0o600)
    print(f'Report: {args.output}', flush=True)


if __name__ == '__main__':
    main()
