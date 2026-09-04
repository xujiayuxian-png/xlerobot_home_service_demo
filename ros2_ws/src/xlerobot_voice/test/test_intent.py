import math

import pytest

from xlerobot_voice.intent import IntentParseError, parse_intent_text


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
