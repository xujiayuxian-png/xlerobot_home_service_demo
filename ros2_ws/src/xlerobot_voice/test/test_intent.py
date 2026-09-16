import math

import pytest

from xlerobot_voice.intent import IntentParseError, parse_intent_text
from xlerobot_voice.chinese_text import simplified_text


def test_traditional_chinese_is_normalized_without_changing_latin_or_numbers():
    assert simplified_text('幫我拿藍色口香糖瓶和 USB-C 2 號打火機') == (
        '帮我拿蓝色口香糖瓶和 USB-C 2 号打火机')


def test_model_output_is_normalized_before_task_dispatch():
    result = parse_intent_text(
        '{"intent":"fetch_deliver","object":"黃色打火機",'
        '"source_place":"桌面","confidence":0.9,'
        '"normalized_command":"幫我拿黃色打火機"}',
        min_confidence=0.6,
    )
    assert result.object_id == '黄色打火机'
    assert result.source_place == 'table'
    assert result.normalized_command == '帮我拿黄色打火机'


def test_plain_json_maps_verified_defaults():
    result = parse_intent_text(
        '{"intent":"fetch_deliver","object":"羽毛球","confidence":0.9}',
        min_confidence=0.6,
    )
    assert result.object_id == '羽毛球'
    assert result.source_place == 'table'
    assert result.recipient_id == 'nearest_person'


def test_fenced_json_keeps_source_but_uses_canonical_recipient():
    result = parse_intent_text(
        '```json\n'
        '{"intent":"fetch_deliver","object":"露营灯",'
        '"source_place":"side_table","recipient":"lisa","confidence":0.8}\n'
        '```',
        min_confidence=0.6,
    )
    assert result.object_id == '露营灯'
    assert result.source_place == 'side_table'
    assert result.recipient_id == 'nearest_person'


@pytest.mark.parametrize('source', ['桌面', '桌子', '桌上', '桌子上', '桌面上', ' table '])
def test_table_display_names_become_navigation_place_id(source):
    result = parse_intent_text(
        '{"intent":"fetch_deliver","object":"打火機",'
        '"source_place":"' + source + '","confidence":0.8}',
        min_confidence=0.6,
    )
    assert result.source_place == 'table'
    assert result.object_id == '打火机'


def test_generic_model_recipient_cannot_break_person_search_contract():
    result = parse_intent_text(
        '{"intent":"fetch_deliver","object":"羽毛球",'
        '"recipient":"person","confidence":0.9}',
        min_confidence=0.6,
    )
    assert result.recipient_id == 'nearest_person'


@pytest.mark.parametrize('heard_name', ['鱼毛球', '鱼模球', '羽模球'])
def test_recurring_asr_homophones_are_corrected(heard_name):
    result = parse_intent_text(
        '{"intent":"fetch_deliver","object":"' + heard_name
        + '","confidence":0.9}',
        min_confidence=0.6,
    )
    assert result.object_id == '羽毛球'


def test_homophone_correction_is_not_an_object_allowlist():
    result = parse_intent_text(
        '{"intent":"fetch_deliver","object":"剪刀","confidence":0.9}',
        min_confidence=0.6,
    )
    assert result.object_id == '剪刀'


@pytest.mark.parametrize(
    'text',
    [
        '好的，我去拿羽毛球',
        '{"intent":"chat","object":"羽毛球","confidence":0.95}',
        '{"intent":"fetch_deliver","object":"拿个","confidence":0.95}',
        '{"intent":"fetch_deliver","object":"羽毛球","confidence":0.2}',
        '{"intent":"fetch_deliver","object":"羽毛球","confidence":NaN}',
    ],
)
def test_unsafe_or_ambiguous_intents_are_rejected(text):
    with pytest.raises(IntentParseError):
        parse_intent_text(text, min_confidence=0.6)


def test_nonfinite_threshold_is_not_a_backend_error():
    with pytest.raises(ValueError, match='min_confidence'):
        parse_intent_text(
            '{"intent":"fetch_deliver","object":"lamp","confidence":0.9}',
            min_confidence=math.nan,
        )
