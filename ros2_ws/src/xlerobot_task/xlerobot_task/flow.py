from __future__ import annotations

from dataclasses import dataclass


CAPABILITY_SEQUENCE = (
    "auto_localize",
    "navigate_to_named_place",
    "detect_object",
    "grasp_object",
    "scan_for_person",
    "approach_target",
    "speak_text",
    "handover_object",
)


@dataclass(frozen=True)
class FetchDeliverRequest:
    object_id: str
    source_place: str
    recipient_id: str
    dry_run: bool


def validate_request(request: FetchDeliverRequest) -> None:
    if not request.object_id.strip():
        raise ValueError("object_id must not be empty")
    if not request.source_place.strip():
        raise ValueError("source_place must not be empty")
    if not request.recipient_id.strip():
        raise ValueError("recipient_id must not be empty")
    if request.recipient_id != "nearest_person":
        raise ValueError("recipient_id must be nearest_person")
