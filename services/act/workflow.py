#!/usr/bin/env python3
"""Reproducible ACT train, qualification, and manifest download helpers."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from hashlib import sha256
import json
import os
from pathlib import Path
import re
import subprocess
import sys

from tools.lib.verify_model_files import (
    DATASET_REPO_ID,
    EXPECTED_STRUCTURE,
    MODEL_REPO_ID,
)


MODEL_ID = "xlerobot-act-local-grasp-v1"
SHA256 = re.compile(r"^[0-9a-f]{64}$")
REVISION = re.compile(r"^[0-9a-f]{40}$")


def train_command(arguments: argparse.Namespace) -> list[str]:
    """Return the pinned LeRobot 0.5.1 command as an argv vector."""
    executable = Path(sys.executable).parent / "lerobot-train"
    command = [str(executable if executable.is_file() else sys.executable)]
    if command[0] == sys.executable:
        command += ["-m", "lerobot.scripts.train"]
    input_features = {
        "observation.state": {"type": "STATE", "shape": [6]},
        "observation.images.wrist": {"type": "VISUAL", "shape": [3, 480, 640]},
    }
    output_features = {"action": {"type": "ACTION", "shape": [6]}}
    command += [
        f"--dataset.repo_id={arguments.repo_id}",
        f"--dataset.root={arguments.dataset}",
        "--policy.type=act",
        f"--output_dir={arguments.output}",
        f"--job_name={arguments.job_name}",
        "--policy.device=cuda",
        "--policy.use_amp=false",
        "--policy.push_to_hub=false",
        "--wandb.enable=false",
        f"--steps={arguments.steps}",
        f"--save_freq={arguments.steps}",
        f"--batch_size={arguments.batch_size}",
        f"--policy.chunk_size={arguments.chunk_size}",
        f"--policy.n_action_steps={arguments.chunk_size}",
        "--policy.vision_backbone=resnet18",
        "--policy.pretrained_backbone_weights=ResNet18_Weights.IMAGENET1K_V1",
        "--policy.dim_model=512",
        "--policy.dim_feedforward=3200",
        "--policy.n_encoder_layers=4",
        "--policy.n_decoder_layers=1",
        "--policy.n_heads=8",
        "--policy.use_vae=true",
        "--policy.latent_dim=32",
        "--policy.input_features=" + json.dumps(input_features, separators=(",", ":")),
        "--policy.output_features=" + json.dumps(output_features, separators=(",", ":")),
        "--save_checkpoint=true",
    ]
    if arguments.resume:
        command.append("--resume=true")
    return command


def qualify(checkpoint: Path, device: str) -> dict:
    """Load a checkpoint and validate the exact deployed feature contract."""
    import torch
    from lerobot.policies.act.modeling_act import ACTPolicy

    if not checkpoint.is_dir():
        raise FileNotFoundError(checkpoint)
    if device.startswith("cuda") and not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable")
    policy = ACTPolicy.from_pretrained(checkpoint, local_files_only=True)
    policy.eval().to(device)
    inputs = policy.config.input_features
    outputs = policy.config.output_features
    state = inputs.get("observation.state")
    action = outputs.get("action")
    if state is None or tuple(state.shape) != (6,):
        raise ValueError("checkpoint observation.state must have shape (6,)")
    if action is None or tuple(action.shape) != (6,):
        raise ValueError("checkpoint action must have shape (6,)")
    images = sorted(key for key in inputs if key.startswith("observation.images."))
    if images != ["observation.images.wrist"]:
        raise ValueError(f"checkpoint camera contract is invalid: {images}")
    if tuple(inputs[images[0]].shape) != (3, 480, 640):
        raise ValueError("checkpoint wrist image must have shape (3, 480, 640)")
    expected = {
        name: value for name, value in EXPECTED_STRUCTURE.items()
        if name not in {"policy_type", "input_features", "output_features"}
    }
    for name, value in expected.items():
        actual = getattr(policy.config, name)
        if name == "pretrained_backbone_weights":
            actual = str(actual)
        if actual != value:
            raise ValueError(
                f"checkpoint {name} is {actual!r}, expected {value!r}"
            )
    structure = {
        **expected,
        "policy_type": "ACT",
        "input_features": {
            "observation.state": list(state.shape),
            images[0]: list(inputs[images[0]].shape),
        },
        "output_features": {"action": list(action.shape)},
    }
    if structure != EXPECTED_STRUCTURE:
        raise ValueError("checkpoint structure differs from the deployment contract")
    return structure


def checkpoint_files(checkpoint: Path, excluded: Path | None = None) -> dict[str, str]:
    """Hash every regular checkpoint file, excluding the manifest being replaced."""
    resolved_excluded = excluded.resolve() if excluded is not None else None
    files: dict[str, str] = {}
    for directory, names, filenames in os.walk(checkpoint, followlinks=False):
        directory_path = Path(directory)
        for name in names:
            path = directory_path / name
            if path.is_symlink():
                raise ValueError(f"checkpoint contains a symlink directory: {path}")
        for name in filenames:
            path = directory_path / name
            if resolved_excluded is not None and path.resolve() == resolved_excluded:
                continue
            if path.is_symlink() or not path.is_file():
                raise ValueError(f"checkpoint contains a non-regular file: {path}")
            files[path.relative_to(checkpoint).as_posix()] = digest(path)
    if not {"config.json", "model.safetensors"}.issubset(files):
        raise ValueError("checkpoint omits config.json or model.safetensors")
    return dict(sorted(files.items()))


def qualification_document(
    checkpoint: Path, output: Path, device: str, structure: dict
) -> dict:
    """Build the self-contained provenance, structure, and file-hash record."""
    return {
        "schema": "xlerobot_act_qualification/v1",
        "status": "passed",
        "model_id": MODEL_ID,
        "license": "Apache-2.0",
        "checked_at": datetime.now(timezone.utc).isoformat(),
        "source": {
            "kind": "local-training",
            "checkpoint_basename": checkpoint.name,
            "dataset_repo_id": DATASET_REPO_ID,
            "lerobot_version": "0.5.1",
            "tool": "tools/act evaluate",
        },
        "structure": structure,
        "qualification": {
            "device": device,
            "scope": "offline deployment qualification; not a grasp success metric",
        },
        "files": checkpoint_files(checkpoint, output),
    }


def write_json_atomic(path: Path, document: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    try:
        temporary.write_text(
            json.dumps(document, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def digest(path: Path) -> str:
    state = sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            state.update(block)
    return state.hexdigest()


def download_model(
    manifest_path: Path, destination: Path, installed_manifest: Path
) -> Path:
    """Fetch one immutable Hub snapshot and verify the public file contract."""
    from huggingface_hub import snapshot_download

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema") != "xlerobot_model_download/v1":
        raise ValueError("unsupported model download manifest")
    repo_id = str(manifest.get("repo_id", ""))
    revision = manifest.get("revision")
    files = manifest.get("files")
    if (
        manifest.get("model_id") != MODEL_ID
        or repo_id != MODEL_REPO_ID
        or not isinstance(files, dict)
        or not files
    ):
        raise ValueError("model download manifest is incomplete")
    if not isinstance(revision, str) or not REVISION.fullmatch(revision):
        raise RuntimeError(
            f"{repo_id} has not been published at an immutable revision yet"
        )
    if manifest.get("availability") != "published":
        raise RuntimeError(f"{repo_id} is not marked published")
    if manifest.get("license") != "Apache-2.0":
        raise ValueError("public ACT checkpoint must be Apache-2.0")
    for name, expected in files.items():
        if (
            not isinstance(name, str)
            or Path(name).is_absolute()
            or ".." in Path(name).parts
            or not isinstance(expected, str)
            or not SHA256.fullmatch(expected)
        ):
            raise ValueError("model manifest contains an invalid path or checksum")
    def verify_downloaded(root: Path) -> bool:
        return all(
            (root / name).is_file()
            and not (root / name).is_symlink()
            and digest(root / name) == expected
            for name, expected in files.items()
        )

    if destination.exists():
        if not destination.is_dir() or destination.is_symlink():
            raise RuntimeError(f"download destination is not a directory: {destination}")
        if not verify_downloaded(destination):
            raise RuntimeError(
                f"{destination} exists but does not match the public manifest; "
                "move it aside and retry"
            )
    else:
        destination.parent.mkdir(parents=True, exist_ok=True)
        import tempfile
        with tempfile.TemporaryDirectory(
            prefix="xlerobot-act-", dir=destination.parent
        ) as temporary:
            staging = Path(temporary) / "checkpoint"
            snapshot_download(
                repo_id=repo_id,
                revision=revision,
                local_dir=staging,
                allow_patterns=sorted(files),
            )
            if not verify_downloaded(staging):
                raise RuntimeError("downloaded checkpoint does not match all file hashes")
            staging.rename(destination)
    write_json_atomic(installed_manifest, manifest)
    return destination


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser()
    commands = root.add_subparsers(dest="command", required=True)

    train = commands.add_parser("train")
    train.add_argument(
        "--dataset", default=os.environ.get("XLEROBOT_ACT_DATASET_ROOT", "")
    )
    train.add_argument(
        "--repo-id", default=os.environ.get("XLEROBOT_ACT_DATASET_REPO_ID", "")
    )
    train.add_argument("--output", default=os.environ.get("XLEROBOT_ACT_OUTPUT", ""))
    train.add_argument("--job-name", default="xlerobot_act_local_grasp_v1")
    train.add_argument("--steps", type=int, default=5000)
    train.add_argument("--batch-size", type=int, default=8)
    train.add_argument("--chunk-size", type=int, default=100)
    train.add_argument("--resume", action="store_true")
    train.add_argument("--dry-run", action="store_true")

    evaluate = commands.add_parser("evaluate")
    evaluate.add_argument(
        "--checkpoint", type=Path,
        default=os.environ.get("XLEROBOT_ACT_CHECKPOINT") or None,
    )
    evaluate.add_argument("--device", default="cuda")
    evaluate.add_argument(
        "--output", type=Path,
        default=os.environ.get("XLEROBOT_ACT_MANIFEST") or None,
    )

    download = commands.add_parser("download")
    download.add_argument("--manifest", type=Path, required=True)
    download.add_argument(
        "--output", type=Path,
        default=os.environ.get("XLEROBOT_ACT_CHECKPOINT") or None,
    )
    download.add_argument(
        "--installed-manifest", type=Path, required=True,
    )
    return root


def main(argv: list[str] | None = None) -> int:
    arguments = parser().parse_args(argv)
    try:
        if arguments.command == "train":
            if not arguments.dataset or not arguments.repo_id or not arguments.output:
                raise ValueError("train requires dataset root, repo ID, and output path")
            if min(arguments.steps, arguments.batch_size, arguments.chunk_size) <= 0:
                raise ValueError("training counts must be positive")
            command = train_command(arguments)
            training_output = Path(arguments.output).expanduser().resolve()
            checkpoint = (
                training_output / "checkpoints" / f"{arguments.steps:06d}"
                / "pretrained_model"
            )
            print(f"training output: {training_output}")
            print(f"expected final checkpoint: {checkpoint}")
            print(
                "qualification command: tools/act evaluate "
                f"--checkpoint {checkpoint} --output {checkpoint / 'model-manifest.json'}"
            )
            if arguments.dry_run:
                print(json.dumps(command, ensure_ascii=False))
                return 0
            return subprocess.run(command, check=False).returncode
        if arguments.command == "evaluate":
            if arguments.checkpoint is None or arguments.output is None:
                raise ValueError("evaluate requires checkpoint and output manifest paths")
            checkpoint = arguments.checkpoint.expanduser().resolve()
            output = arguments.output.expanduser().resolve()
            structure = qualify(checkpoint, arguments.device)
            report = qualification_document(
                checkpoint, output, arguments.device, structure
            )
            write_json_atomic(output, report)
            print(f"qualified checkpoint: {checkpoint}")
            print(f"qualification manifest: {output}")
            return 0
        if arguments.output is None:
            raise ValueError("download requires an output directory")
        output = download_model(
            arguments.manifest.expanduser().resolve(),
            arguments.output.expanduser().resolve(),
            arguments.installed_manifest.expanduser().resolve(),
        )
        print(f"downloaded checkpoint: {output}")
        print(f"installed public manifest: {arguments.installed_manifest.resolve()}")
        return 0
    except (FileNotFoundError, json.JSONDecodeError, OSError, RuntimeError, ValueError) as exc:
        raise SystemExit(str(exc)) from exc


if __name__ == "__main__":
    raise SystemExit(main())
