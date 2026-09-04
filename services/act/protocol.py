"""Dependency-light validation for the ACT inference HTTP boundary."""

from __future__ import annotations

import hmac
import math


STATE_DIMENSION = 6
MAX_ACTION_STEPS = 500


class ProtocolError(ValueError):
    """A client-visible request or response contract failure."""


def bearer_token(authorization: str | None) -> str:
    """Extract one bearer token without accepting alternate auth schemes."""
    if not authorization:
        return ''
    scheme, separator, token = authorization.partition(' ')
    if separator != ' ' or scheme.lower() != 'bearer' or not token.strip():
        return ''
    return token.strip()


def authorized(authorization: str | None, expected_token: str) -> bool:
    """Compare credentials in constant time."""
    supplied = bearer_token(authorization)
    return bool(expected_token and supplied) and hmac.compare_digest(
        supplied, expected_token
    )


def finite_vector(values, *, size: int, field: str) -> list[float]:
    """Return an exact-size finite float vector."""
    if not isinstance(values, (list, tuple)) or len(values) != size:
        raise ProtocolError(f'{field} must contain exactly {size} values')
    try:
        result = [float(value) for value in values]
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f'{field} contains a nonnumeric value') from exc
    if not all(math.isfinite(value) for value in result):
        raise ProtocolError(f'{field} contains a nonfinite value')
    return result


def validate_request(
    payload,
    *,
    image_fields=('head_image_base64', 'wrist_image_base64'),
) -> dict:
    """Validate the JSON envelope before image decoding or model access."""
    if not isinstance(payload, dict):
        raise ProtocolError('request body must be a JSON object')
    supported = {'head_image_base64', 'wrist_image_base64'}
    if not image_fields or not set(image_fields) <= supported:
        raise ValueError('image_fields must be a nonempty supported subset')
    required = {*image_fields, 'state'}
    missing = sorted(required - payload.keys())
    if missing:
        raise ProtocolError(f'missing request field(s): {", ".join(missing)}')
    images = {}
    for key in image_fields:
        value = payload[key]
        if not isinstance(value, str) or not value:
            raise ProtocolError(f'{key} must be a nonempty base64 string')
        images[key] = value
    return {
        **images,
        'state': finite_vector(
            payload['state'], size=STATE_DIMENSION, field='state'
        ),
    }


def validate_actions(actions) -> list[list[float]]:
    """Validate one bounded physical-unit action chunk."""
    if not isinstance(actions, (list, tuple)) or not actions:
        raise ProtocolError('model returned an empty action chunk')
    if len(actions) > MAX_ACTION_STEPS:
        raise ProtocolError(
            f'model returned more than {MAX_ACTION_STEPS} action steps'
        )
    return [
        finite_vector(row, size=STATE_DIMENSION, field=f'action[{index}]')
        for index, row in enumerate(actions)
    ]

