#!/usr/bin/env python3
"""Download the two pinned local voice models and verify every runtime file."""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
from pathlib import Path
import shutil
import tarfile
import tempfile
from urllib.request import urlopen


def load_manifest(path: Path) -> dict:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if manifest.get("schema") != "xlerobot_voice_models/v1":
        raise ValueError("unsupported voice model manifest")
    for section in ("kws", "whisper"):
        record = manifest.get(section)
        files = record.get("files") if isinstance(record, dict) else None
        if not isinstance(files, dict) or not files:
            raise ValueError(f"voice model manifest has no {section} files")
        for name, expected in files.items():
            if (
                not isinstance(name, str)
                or Path(name).name != name
                or not isinstance(expected, str)
                or len(expected) != 64
                or set(expected) - set("0123456789abcdef")
            ):
                raise ValueError(f"invalid {section} model path or checksum")
    revision = manifest["whisper"].get("revision")
    if (
        not isinstance(revision, str)
        or len(revision) != 40
        or set(revision) - set("0123456789abcdef")
    ):
        raise ValueError("Whisper revision must be an immutable commit")
    return manifest


def digest(path: Path) -> str:
    state = sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            state.update(block)
    return state.hexdigest()


def verify_files(directory: Path, files: dict[str, str]) -> list[str]:
    errors = []
    for name, expected in files.items():
        path = directory / name
        if not path.is_file():
            errors.append(f"missing {path}")
        elif digest(path) != expected:
            errors.append(f"checksum mismatch for {path}")
    return errors


def ensure_empty_destination(destination: Path) -> None:
    if destination.exists():
        raise RuntimeError(
            f"{destination} exists but does not match the pinned model; "
            "move it aside and rerun setup"
        )
    destination.parent.mkdir(parents=True, exist_ok=True)


def install_kws(record: dict, destination: Path) -> None:
    files = record["files"]
    if not verify_files(destination, files):
        print(f"KWS model already verified: {destination}")
        return
    ensure_empty_destination(destination)
    with tempfile.TemporaryDirectory(
        prefix="xlerobot-kws-", dir=destination.parent
    ) as temporary:
        temporary_path = Path(temporary)
        archive = temporary_path / "model.tar.bz2"
        with urlopen(record["source_url"], timeout=60) as response, archive.open(
            "wb"
        ) as output:
            shutil.copyfileobj(response, output)
        staging = temporary_path / "model"
        staging.mkdir()
        with tarfile.open(archive, "r:bz2") as bundle:
            members = [member for member in bundle.getmembers() if member.isfile()]
            for name in files:
                matches = [member for member in members if Path(member.name).name == name]
                if len(matches) != 1:
                    raise RuntimeError(
                        f"KWS archive must contain exactly one {name}; found {len(matches)}"
                    )
                source = bundle.extractfile(matches[0])
                if source is None:
                    raise RuntimeError(f"cannot read {name} from the KWS archive")
                with source, (staging / name).open("wb") as output:
                    shutil.copyfileobj(source, output)
        errors = verify_files(staging, files)
        if errors:
            raise RuntimeError("; ".join(errors))
        staging.rename(destination)
    print(f"KWS model downloaded and verified: {destination}")


def install_whisper(record: dict, destination: Path) -> None:
    files = record["files"]
    if not verify_files(destination, files):
        print(f"Whisper model already verified: {destination}")
        return
    ensure_empty_destination(destination)
    try:
        from huggingface_hub import snapshot_download
    except ImportError as exc:
        raise RuntimeError("huggingface-hub is required to download Whisper") from exc
    with tempfile.TemporaryDirectory(
        prefix="xlerobot-whisper-", dir=destination.parent
    ) as temporary:
        staging = Path(temporary) / "model"
        snapshot_download(
            repo_id=record["repo_id"],
            revision=record["revision"],
            local_dir=staging,
            allow_patterns=sorted(files),
        )
        errors = verify_files(staging, files)
        if errors:
            raise RuntimeError("; ".join(errors))
        staging.rename(destination)
    print(f"Whisper model downloaded and verified: {destination}")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--kws-dir", type=Path, required=True)
    parser.add_argument("--whisper-dir", type=Path, required=True)
    parser.add_argument("--check", action="store_true")
    parser.add_argument(
        "--component", choices=("all", "kws", "whisper"), default="all"
    )
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    try:
        manifest = load_manifest(arguments.manifest)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise SystemExit(str(exc)) from exc
    records = {
        "KWS": (arguments.kws_dir, manifest["kws"]["files"]),
        "Whisper": (arguments.whisper_dir, manifest["whisper"]["files"]),
    }
    if arguments.component != "all":
        selected = "KWS" if arguments.component == "kws" else "Whisper"
        records = {selected: records[selected]}
    if arguments.check:
        errors = [
            error
            for label, (directory, files) in records.items()
            for error in verify_files(directory, files)
        ]
        if errors:
            raise SystemExit("voice model verification failed: " + "; ".join(errors))
        print("voice models verified")
        return 0
    try:
        if arguments.component in {"all", "kws"}:
            install_kws(manifest["kws"], arguments.kws_dir)
        if arguments.component in {"all", "whisper"}:
            install_whisper(manifest["whisper"], arguments.whisper_dir)
    except (OSError, RuntimeError, tarfile.TarError) as exc:
        raise SystemExit(str(exc)) from exc
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
