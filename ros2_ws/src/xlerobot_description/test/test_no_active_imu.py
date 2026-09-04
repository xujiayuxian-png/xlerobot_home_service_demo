from pathlib import Path
import re


def test_workspace_has_no_active_imu_or_ekf_dependency():
    """Keep the reference runtime on lidar plus wheel odometry only."""
    workspace_src = Path(__file__).resolve().parents[2]
    forbidden = re.compile(
        r'(?i)(/imu(?:\W|$)|imu_link|imu_topic|robot_localization|\bekf(?:\W|$))'
    )
    violations = []
    for path in workspace_src.rglob('*'):
        if not path.is_file() or path.suffix not in {
            '.cpp', '.hpp', '.h', '.py', '.xml', '.yaml', '.yml', '.xacro'
        }:
            continue
        if '__pycache__' in path.parts or 'test' in path.parts:
            continue
        match = forbidden.search(path.read_text(encoding='utf-8', errors='ignore'))
        if match:
            violations.append(f'{path.relative_to(workspace_src)}: {match.group(0)}')
    assert violations == []
