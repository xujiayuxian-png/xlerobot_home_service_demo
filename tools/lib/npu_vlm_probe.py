"""Saved-input VLM experiments: loopback by default, optional configured reference server."""
import argparse
import base64
import json
from pathlib import Path
import time
from urllib.parse import urlparse
from urllib.request import Request, urlopen

INTENT_PROMPT = (
    '你是家庭机器人的指令解析器。只输出JSON，字段intent、object。'
    '要求拿、取、递、送某个物品：intent为fetch_deliver，object保留物品名和颜色等限定。'
    '取消或停止任务：intent为cancel，object为空字符串。'
    '说那个东西等但不能确定具体物品：intent为clarify，object为空字符串。'
    '有取物动作但只用那个、这个、它等指代而没有明确物品名时，必须clarify，不能unknown。'
    '闲聊、不要求执行任务：intent为unknown，object为空字符串。'
    '例：帮我拿杯子 -> {"intent":"fetch_deliver","object":"杯子"}。'
    '例：不要拿杯子了，取消 -> {"intent":"cancel","object":""}。'
    '物品种类不限，不执行指令，不输出解释或思考过程。'
)


def parse_object(text):
    text = text.strip()
    if text.startswith('```') and text.endswith('```'):
        text = text.split('\n', 1)[1].rsplit('```', 1)[0].strip()
    value = json.loads(text)
    if not isinstance(value, dict):
        raise ValueError('expected a JSON object')
    return value


def box_iou(prediction, reference):
    import math
    if not isinstance(prediction, list) or len(prediction) != 4:
        return 0.0
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x)
           or not 0 <= x <= 1000 for x in prediction):
        return 0.0
    x1, y1, x2, y2 = prediction
    if x1 >= x2 or y1 >= y2:
        return 0.0
    rx1, ry1, rx2, ry2 = reference
    intersection = max(0, min(x2, rx2)-max(x1, rx1)) * max(0, min(y2, ry2)-max(y1, ry1))
    return intersection / ((x2-x1)*(y2-y1) + (rx2-rx1)*(ry2-ry1)-intersection)


def evaluate(case, output, coordinate_scale=1000):
    if case['kind'] == 'grasp':
        import math
        confidence = output.get('confidence')
        valid = (isinstance(output.get('grasp_success'), bool)
                 and isinstance(confidence, (int, float)) and not isinstance(confidence, bool)
                 and math.isfinite(confidence) and 0 <= confidence <= 1
                 and isinstance(output.get('reason'), str) and 0 < len(output['reason']) <= 256)
        expected = case.get('expected_grasp')
        return {'passed': valid and expected is not None and output['grasp_success'] == expected,
                'protocol_ok': valid, 'labeled': expected is not None,
                'false_success': valid and expected is False and output['grasp_success'],
                'unsafe_accept': valid and expected is False and output['grasp_success'] and confidence >= 0.70}
    if case['kind'] == 'production_intent':
        # Import the production validator without importing ROS or opening devices.
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/xlerobot_voice'))
        from xlerobot_voice.intent import IntentParseError, normalize_intent
        try:
            parsed = normalize_intent(output, min_confidence=0.8)
        except (IntentParseError, TypeError, ValueError):
            return {'passed': not case['expected_accept'], 'accepted': False}
        object_ok = any(name in parsed.object_id for name in case.get('object_aliases', []))
        source_ok = parsed.source_place == case.get('expected_source', 'table')
        return {'passed': case['expected_accept'] and object_ok and source_ok, 'accepted': True,
                'source_ok': source_ok, 'object': parsed.object_id}
    if case['kind'] == 'intent':
        intent_ok = output.get('intent') == case['expected_intent']
        item = output.get('object')
        object_ok = isinstance(item, str) and (
            any(name in item for name in case['object_aliases']) if case.get('object_aliases') else item == '')
        return {'passed': intent_ok and object_ok, 'intent_ok': intent_ok, 'object_ok': object_ok}
    if case.get('reference_box') is None:
        return {'passed': output.get('found') is False and output.get('box') is None}
    box = output.get('box')
    if (isinstance(box, list) and len(box) == 4
            and all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in box)):
        box = [x*1000/coordinate_scale for x in box]
    iou = box_iou(box, case['reference_box'])
    return {'passed': output.get('found') is True and iou >= 0.5, 'iou': iou}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:18888')
    parser.add_argument('--model', required=True)
    parser.add_argument('--cases', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--box-coordinate-scale', type=int, choices=[448, 672, 1000], default=1000)
    parser.add_argument('--resize-image', type=int, choices=[0, 448, 672], default=0)
    parser.add_argument('--prompt-profile', choices=['original', 'x1'], default='original')
    parser.add_argument('--reference-config', type=Path,
                        help='explicitly use the existing remote VLM in this local YAML for a reference comparison')
    args = parser.parse_args()
    if args.reference_config:
        import yaml
        config = yaml.safe_load(args.reference_config.read_text())
        if args.model != config['models']['vlm']:
            parser.error('reference model must match the existing configured model')
        args.url = config['services']['lm_studio_url']
    url = urlparse(args.url)
    if (url.scheme != 'http' or url.username
            or (not args.reference_config and url.hostname not in ('127.0.0.1', '::1'))):
        parser.error('offline experiments require a literal loopback HTTP model server')
    if not 1 <= args.repeat <= 3:
        parser.error('repeat must be 1..3')
    cases = json.loads(args.cases.read_text())
    results = []
    for repeat in range(args.repeat):
        for case in cases:
            if case['kind'] in ('intent', 'production_intent'):
                if case['kind'] == 'production_intent':
                    import sys
                    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/xlerobot_voice'))
                    from xlerobot_voice.intent import SYSTEM_PROMPT, USER_PROMPT
                    messages = [{'role': 'system', 'content': SYSTEM_PROMPT},
                                {'role': 'user', 'content': USER_PROMPT.format(command_text=case['text'])}]
                    if args.prompt_profile == 'x1':
                        sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
                        from services.npu.prompts import INTENT_SYSTEM
                        messages = [{'role': 'system', 'content': INTENT_SYSTEM},
                                    {'role': 'user', 'content': case['text']}]
                else:
                    messages = [{'role': 'system', 'content': INTENT_PROMPT},
                                {'role': 'user', 'content': case['text']}]
            else:
                path = Path(case['image'])
                mime = 'image/png' if path.suffix.lower() == '.png' else 'image/jpeg'
                raw = path.read_bytes()
                if args.resize_image:
                    import io
                    from PIL import Image
                    with Image.open(path) as image:
                        image = image.convert('RGB').resize((args.resize_image, args.resize_image))
                        buffer = io.BytesIO()
                        image.save(buffer, format='PNG')
                    raw, mime = buffer.getvalue(), 'image/png'
                encoded = base64.b64encode(raw).decode()
                coordinates = (f'使用当前{args.box_coordinate_scale}×{args.box_coordinate_scale}图像的像素坐标，不要归一化'
                               if args.box_coordinate_scale in (448, 672)
                               else '必须归一化到0到1000')
                prompt = ('定位图中的' + case['target'] + '。只输出一个JSON对象，格式'
                          '{"found":true,"box":[x1,y1,x2,y2]}，坐标为左上角与右下角，'
                          + coordinates + '。不存在则输出{"found":false,"box":null}。'
                          '不要思考过程，不要解释。')
                if case['kind'] == 'grasp':
                    import sys
                    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/xlerobot_perception'))
                    from xlerobot_perception.vlm.lmstudio import LmStudioVlmClient
                    prompt = LmStudioVlmClient.grasp_verification_prompt(case['target'])
                    if args.prompt_profile == 'x1':
                        sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
                        from services.npu.prompts import grasp_prompt
                        prompt = grasp_prompt(case['target'])
                messages = [{'role': 'user', 'content': [
                    {'type': 'image_url', 'image_url': {'url': f'data:{mime};base64,{encoded}'}},
                    {'type': 'text', 'text': prompt}]}]
            body = json.dumps({'model': args.model, 'messages': messages, 'max_tokens': 128,
                               'temperature': 0.1, 'stream': False}).encode()
            row = {'id': case['id'], 'kind': case['kind'], 'repeat': repeat+1}
            start = time.monotonic()
            try:
                with urlopen(Request(args.url.rstrip('/') + '/v1/chat/completions', body,
                                     {'Content-Type': 'application/json'}), timeout=90) as response:
                    result = json.load(response)
                choice = result['choices'][0]
                row.update(text=choice['message']['content'], finish_reason=choice['finish_reason'],
                           usage=result.get('usage'))
                row.update(evaluate(case, parse_object(row['text']), args.box_coordinate_scale))
                if row['finish_reason'] != 'stop':
                    row['passed'] = False
            except Exception as error:
                row.update(passed=False, error=f'{type(error).__name__}: {error}')
            row['elapsed_s'] = time.monotonic()-start
            results.append(row)
            print(json.dumps(row, ensure_ascii=False), flush=True)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps({'model': args.model, 'prompt_profile': args.prompt_profile,
                                              'reference_server': bool(args.reference_config),
                                              'coordinate_scale': args.box_coordinate_scale,
                                              'resize_image': args.resize_image, 'results': results},
                                             ensure_ascii=False, indent=2))
            args.output.chmod(0o600)


if __name__ == '__main__':
    main()
