"""Locate calibration profiles in source and installed workspaces."""

from __future__ import annotations

from pathlib import Path


def config_dir() -> Path:
    """Return the package calibration configuration directory."""
    source = Path(__file__).resolve().parents[1] / 'config'
    if source.is_dir():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory('xlerobot_calibration_tools')) / 'config'
    except (ImportError, LookupError):
        installed = source
    if not installed.is_dir():
        raise FileNotFoundError('xlerobot_calibration_tools calibration config is missing')
    return installed


def default_config(name: str) -> Path:
    path = config_dir() / name
    if not path.is_file():
        raise FileNotFoundError(f'calibration config is missing: {path}')
    return path
