from types import SimpleNamespace

import pytest
from sensor_msgs.msg import Image
from xlerobot_interfaces.msg import CapabilityError
from xlerobot_perception import verify_grasp_node as module


@pytest.mark.parametrize('answers,expected_calls,expected_grasped', [
    ([(False, .8), (True, .9)], 2, True),
    ([(True, .9), (False, .8)], 1, True),
    ([(False, .8), (False, .8)], 2, False),
    ([(True, .5), (True, .9)], 2, True),
])
def test_recheck_uses_new_frame_and_never_promotes_two_negatives(
        monkeypatch, answers, expected_calls, expected_grasped):
    images = [Image(), Image()]
    for n, image in enumerate(images):
        image.header.stamp.sec = n + 1
    captured, queried = [], []

    def fresh(_goal):
        image = images[len(captured)]
        captured.append(image)
        return image

    def verify(image, target):
        assert target == '羽毛球'
        grasped, confidence = answers[len(queried)]
        queried.append(image)
        return SimpleNamespace(grasped=grasped, confidence=confidence, reason='test'), .1

    monkeypatch.setattr(module, 'color_to_bgr', lambda image: image)
    node = SimpleNamespace(vlm=SimpleNamespace(backend='npu', verify_grasp=verify),
                           success_confidence=.7, _fresh_image=fresh,
                           _feedback=lambda *args: None,
                           get_logger=lambda: SimpleNamespace(info=lambda _: None))
    goal = SimpleNamespace(is_cancel_requested=False,
                           request=SimpleNamespace(object_id='羽毛球'))
    result, latency = module.VerifyGraspNode._verify_images(node, goal)
    assert result.grasped is expected_grasped
    assert len(captured) == len(queried) == expected_calls
    assert queried[0] is images[0]
    if expected_calls == 2:
        assert queried[1] is images[1]
    assert latency == pytest.approx(.1 * expected_calls)


def test_cancel_during_inference_does_not_start_second_view(monkeypatch):
    goal = SimpleNamespace(is_cancel_requested=False,
                           request=SimpleNamespace(object_id='羽毛球'))

    def verify(*_args):
        goal.is_cancel_requested = True
        return SimpleNamespace(grasped=True, confidence=.9, reason='test'), .1

    monkeypatch.setattr(module, 'color_to_bgr', lambda image: image)
    node = SimpleNamespace(vlm=SimpleNamespace(backend='npu', verify_grasp=verify),
                           _fresh_image=lambda _: Image(), _feedback=lambda *args: None)
    with pytest.raises(module.VerificationFailure) as failure:
        module.VerifyGraspNode._verify_images(node, goal)
    assert failure.value.code == CapabilityError.CANCELED
