#!/usr/bin/env python3
"""Generate exact-size printable SVG and PDF targets from one profile."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from cv2 import aruco
import numpy as np
import yaml


MM_TO_PT = 72.0 / 25.4


def _dictionary():
    if hasattr(aruco, 'getPredefinedDictionary'):
        return aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    return aruco.Dictionary_get(aruco.DICT_APRILTAG_36h11)


def marker_cells(marker_id: int) -> np.ndarray:
    image = np.zeros((8, 8), dtype=np.uint8)
    if hasattr(aruco, 'generateImageMarker'):
        aruco.generateImageMarker(_dictionary(), marker_id, 8, image, 1)
    else:
        aruco.drawMarker(_dictionary(), marker_id, 8, image, 1)
    return image


def _marker_svg(marker_id: int, x_mm: float, y_mm: float, size_mm: float) -> list[str]:
    cells = marker_cells(marker_id)
    module = size_mm / 8.0
    rows = [f'  <g id="tag-{marker_id}" shape-rendering="crispEdges">']
    for row in range(8):
        for column in range(8):
            if int(cells[row, column]) == 0:
                rows.append(
                    f'    <rect x="{x_mm + column * module:.6g}" '
                    f'y="{y_mm + row * module:.6g}" width="{module:.6g}" '
                    f'height="{module:.6g}" fill="#000"/>'
                )
    rows.append('  </g>')
    return rows


def _page(width_mm: float, height_mm: float, body: list[str], title: str) -> str:
    return '\n'.join([
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width_mm}mm" '
        f'height="{height_mm}mm" viewBox="0 0 {width_mm} {height_mm}">',
        f'  <title>{title}</title>',
        '  <rect width="100%" height="100%" fill="#fff"/>',
        *body,
        '</svg>',
        '',
    ])


def _pdf(
    width_mm: float,
    height_mm: float,
    markers: list[tuple[int, float, float, float]],
    title: str,
) -> bytes:
    """Return a deterministic, uncompressed, vector-only PDF."""
    commands = ['0 g']
    for marker_id, x_mm, y_mm, size_mm in markers:
        cells = marker_cells(marker_id)
        module_mm = size_mm / 8.0
        module_pt = module_mm * MM_TO_PT
        for row in range(8):
            for column in range(8):
                if int(cells[row, column]) != 0:
                    continue
                x_pt = (x_mm + column * module_mm) * MM_TO_PT
                # SVG/image rows grow down; PDF coordinates grow up.
                y_pt = (
                    height_mm - y_mm - (row + 1) * module_mm
                ) * MM_TO_PT
                commands.append(
                    f'{x_pt:.6f} {y_pt:.6f} '
                    f'{module_pt:.6f} {module_pt:.6f} re f'
                )
    stream = ('\n'.join(commands) + '\n').encode('ascii')
    width_pt = width_mm * MM_TO_PT
    height_pt = height_mm * MM_TO_PT
    objects = [
        b'<< /Type /Catalog /Pages 2 0 R >>',
        b'<< /Type /Pages /Kids [3 0 R] /Count 1 >>',
        (
            f'<< /Type /Page /Parent 2 0 R '
            f'/MediaBox [0 0 {width_pt:.6f} {height_pt:.6f}] '
            '/Resources << >> /Contents 4 0 R >>'
        ).encode('ascii'),
        f'<< /Length {len(stream)} >>\nstream\n'.encode('ascii')
        + stream
        + b'endstream',
        (
            f'<< /Title ({title}) '
            '/Producer (xlerobot deterministic target generator) >>'
        ).encode('ascii'),
    ]
    result = bytearray(b'%PDF-1.4\n% deterministic vector target\n')
    offsets = [0]
    for number, value in enumerate(objects, start=1):
        offsets.append(len(result))
        result.extend(f'{number} 0 obj\n'.encode('ascii'))
        result.extend(value)
        result.extend(b'\nendobj\n')
    xref = len(result)
    result.extend(f'xref\n0 {len(objects) + 1}\n'.encode('ascii'))
    result.extend(b'0000000000 65535 f \n')
    for offset in offsets[1:]:
        result.extend(f'{offset:010d} 00000 n \n'.encode('ascii'))
    result.extend(
        (
            f'trailer\n<< /Size {len(objects) + 1} /Root 1 0 R '
            '/Info 5 0 R >>\n'
            f'startxref\n{xref}\n%%EOF\n'
        ).encode('ascii')
    )
    return bytes(result)


def _head_layout(profile: dict) -> tuple[float, float, list[tuple[int, float, float, float]]]:
    size = float(profile['tag_size_m']) * 1000.0
    gap = float(profile['tag_separation_m']) * 1000.0
    rows = int(profile['rows'])
    columns = int(profile['columns'])
    board_width = columns * size + (columns - 1) * gap
    board_height = rows * size + (rows - 1) * gap
    page_width, page_height = 297.0, 210.0
    x0 = (page_width - board_width) / 2.0
    y0 = (page_height - board_height) / 2.0
    markers = []
    for row in range(rows):
        for column in range(columns):
            markers.append((
                int(profile['first_id']) + row * columns + column,
                x0 + column * (size + gap),
                y0 + row * (size + gap),
                size,
            ))
    return page_width, page_height, markers


def _handeye_layout(
    profile: dict,
) -> tuple[float, float, list[tuple[int, float, float, float]]]:
    size = float(profile['tag_size_m']) * 1000.0
    page_width, page_height = 210.0, 297.0
    markers = [(
        int(profile['marker_id']),
        (page_width - size) / 2.0,
        (page_height - size) / 2.0,
        size,
    )]
    return page_width, page_height, markers


def render_head(profile: dict) -> str:
    page_width, page_height, markers = _head_layout(profile)
    body = []
    for marker in markers:
        body.extend(_marker_svg(*marker))
    return _page(page_width, page_height, body, 'AprilTag 36h11 head board IDs 0-15')


def render_handeye(profile: dict) -> str:
    page_width, page_height, markers = _handeye_layout(profile)
    return _page(
        page_width,
        page_height,
        _marker_svg(*markers[0]),
        'AprilTag 36h11 hand-eye target ID 23',
    )


def render_head_pdf(profile: dict) -> bytes:
    page_width, page_height, markers = _head_layout(profile)
    return _pdf(
        page_width, page_height, markers,
        'AprilTag 36h11 head board IDs 0-15',
    )


def render_handeye_pdf(profile: dict) -> bytes:
    page_width, page_height, markers = _handeye_layout(profile)
    return _pdf(
        page_width, page_height, markers,
        'AprilTag 36h11 hand-eye target ID 23',
    )


def main() -> None:
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, default=package / 'config/targets.yaml')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument(
        '--stdout', choices=('head', 'handeye', 'head-pdf', 'handeye-pdf')
    )
    args = parser.parse_args()
    document = yaml.safe_load(args.config.read_text(encoding='utf-8'))
    if document.get('schema') != 'xlerobot_calibration_targets/v1':
        raise SystemExit('unsupported target config schema')
    targets = document['targets']
    outputs = {
        'head': render_head(targets['head_camera']),
        'handeye': render_handeye(targets['right_handeye']),
        'head-pdf': render_head_pdf(targets['head_camera']),
        'handeye-pdf': render_handeye_pdf(targets['right_handeye']),
    }
    if args.stdout:
        value = outputs[args.stdout]
        if isinstance(value, bytes):
            sys.stdout.buffer.write(value)
        else:
            print(value, end='')
        return
    if args.output_dir is None:
        parser.error('--output-dir is required unless --stdout is used')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / 'head_4x4_ids_0-15_40mm.svg').write_text(
        outputs['head'], encoding='utf-8'
    )
    (args.output_dir / 'handeye_tag23_60mm.svg').write_text(
        outputs['handeye'], encoding='utf-8'
    )
    (args.output_dir / 'head_4x4_ids_0-15_40mm.pdf').write_bytes(
        outputs['head-pdf']
    )
    (args.output_dir / 'handeye_tag23_60mm.pdf').write_bytes(
        outputs['handeye-pdf']
    )


if __name__ == '__main__':
    main()
