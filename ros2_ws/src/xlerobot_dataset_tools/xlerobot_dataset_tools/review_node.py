"""Store review sidecars without modifying immutable raw episodes."""

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re

import rclpy
from rclpy.node import Node
from xlerobot_interfaces.msg import CapabilityError
from xlerobot_interfaces.srv import ReviewEpisode

from .gpu_cli import validate_raw_episode


IDENTIFIER = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,95}$')


class ReviewNode(Node):
    def __init__(self):
        super().__init__('episode_review')
        self.root = Path(str(self.declare_parameter(
            'artifact_root', '.xlerobot/artifacts'
        ).value)).expanduser().resolve()
        self.create_service(ReviewEpisode, '/episodes/review', self.review)

    def review(self, request, response):
        try:
            if not IDENTIFIER.fullmatch(request.dataset_id):
                raise ValueError('invalid dataset_id')
            if not IDENTIFIER.fullmatch(request.episode_id):
                raise ValueError('invalid episode_id')
            if request.status not in {'accepted', 'rejected'}:
                raise ValueError('review status must be accepted or rejected')
            dataset = self.root / 'datasets' / request.dataset_id
            episode = dataset / 'raw' / request.episode_id
            validate_raw_episode(episode, request.episode_id)
            reviews = dataset / 'reviews'
            if reviews.is_symlink():
                raise ValueError('review directory must not be a symlink')
            reviews.mkdir(parents=True, exist_ok=True)
            path = reviews / f'{request.episode_id}.json'
            requested = {
                'schema': 'xlerobot_episode_review/v1',
                'episode_id': request.episode_id, 'status': request.status,
                'failure_reason': request.failure_reason, 'notes': request.notes,
                'operator': request.operator_id,
            }
            if path.exists() or path.is_symlink():
                if path.is_symlink() or not path.is_file():
                    raise ValueError('existing review is not a regular file')
                existing = json.loads(path.read_text(encoding='utf-8'))
                comparable = {
                    key: existing.get(key) for key in requested
                } if isinstance(existing, dict) else {}
                if comparable != requested:
                    raise ValueError(
                        'episode review is immutable and already differs'
                    )
                response.error.code = CapabilityError.NONE
                response.error.message = 'identical episode review already exists'
                response.review_uri = path.as_uri()
                return response
            document = {
                **requested,
                'reviewed_at': datetime.now(timezone.utc).isoformat(),
            }
            temporary = path.with_name(
                f'.{path.name}.{os.getpid()}.tmp'
            )
            temporary.write_text(
                json.dumps(
                    document, ensure_ascii=False, indent=2
                ) + '\n',
                encoding='utf-8',
            )
            os.replace(temporary, path)
            response.error.code = CapabilityError.NONE
            response.error.message = 'episode review saved'
            response.review_uri = path.as_uri()
        except Exception as error:
            response.error.code = CapabilityError.INVALID_GOAL
            response.error.message = str(error)
        return response


def main():
    rclpy.init()
    node = ReviewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
