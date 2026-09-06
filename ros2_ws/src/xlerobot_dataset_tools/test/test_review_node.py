import json

from xlerobot_dataset_tools import review_node
from xlerobot_dataset_tools.review_node import ReviewNode
from xlerobot_interfaces.msg import CapabilityError
from xlerobot_interfaces.srv import ReviewEpisode


def test_review_contract_is_dataset_scoped_and_idempotent(tmp_path, monkeypatch):
    request = ReviewEpisode.Request()
    assert hasattr(request, 'dataset_id')
    assert hasattr(request, 'episode_id')
    assert not hasattr(request, 'episode_uri')

    manifest = tmp_path / 'datasets/dataset-001/raw/episode-001/manifest.json'
    manifest.parent.mkdir(parents=True)
    manifest.write_text('{}\n', encoding='utf-8')
    monkeypatch.setattr(review_node, 'validate_raw_episode', lambda *_: None)

    node = object.__new__(ReviewNode)
    node.root = tmp_path
    request.dataset_id = 'dataset-001'
    request.episode_id = 'episode-001'
    request.status = 'accepted'
    request.notes = 'usable'
    request.operator_id = 'operator'
    response = ReviewNode.review(node, request, ReviewEpisode.Response())

    assert response.error.code == CapabilityError.NONE
    sidecar = tmp_path / 'datasets/dataset-001/reviews/episode-001.json'
    document = json.loads(sidecar.read_text(encoding='utf-8'))
    assert document['episode_id'] == 'episode-001'
    assert document['status'] == 'accepted'

    first_bytes = sidecar.read_bytes()
    repeated = ReviewNode.review(node, request, ReviewEpisode.Response())
    assert repeated.error.code == CapabilityError.NONE
    assert sidecar.read_bytes() == first_bytes

    request.notes = 'changed'
    request.status = 'rejected'
    changed = ReviewNode.review(node, request, ReviewEpisode.Response())
    assert changed.error.code == CapabilityError.NONE
    assert json.loads(sidecar.read_text())['status'] == 'rejected'
    assert manifest.read_text() == '{}\n'


def test_review_rejects_incomplete_or_invalid_raw_episode(tmp_path):
    manifest = tmp_path / 'datasets/dataset-001/raw/episode-001/manifest.json'
    manifest.parent.mkdir(parents=True)
    manifest.write_text('{}\n', encoding='utf-8')
    node = object.__new__(ReviewNode)
    node.root = tmp_path
    request = ReviewEpisode.Request(
        dataset_id='dataset-001',
        episode_id='episode-001',
        status='accepted',
        operator_id='operator',
    )

    response = ReviewNode.review(node, request, ReviewEpisode.Response())
    assert response.error.code == CapabilityError.INVALID_GOAL
    assert not (tmp_path / 'datasets/dataset-001/reviews').exists()
