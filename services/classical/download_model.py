#!/usr/bin/env python3
"""Download and checksum the exact SAM2 checkpoint used by this demo."""

from __future__ import annotations

import json

import sam2_prompt


def main() -> int:
    path = sam2_prompt.resolve_checkpoint(allow_download=True)
    print(json.dumps({
        "model_id": sam2_prompt.MODEL_ID,
        "revision": sam2_prompt.MODEL_REVISION,
        "checkpoint": str(path),
        "sha256": sam2_prompt.MODEL_SHA256,
    }))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
