import base64
from pathlib import Path

import numpy as np
import pytest
import yaml

from xlerobot_perception.detection.seg_instances import decode_prompted_instance
from xlerobot_perception.grasp.gpd_client import GpdCandidate
from xlerobot_perception.grasp.refine_cloud import (
    RefinementConfig,
    refine_object_cloud,
)
from xlerobot_perception.grasp.runtime_calibration import (
    load_classical_runtime_calibration,
)
from xlerobot_perception.grasp.top_grasp import (
    TablePlane,
    TopGraspConfig,
    centroid_top_grasp,
    gpd_top_grasp,
    masked_point_cloud,
    transform_cloud,
)


FIXTURE = (
    Path(__file__).resolve().parents[4]
    / 'examples/grasping/shared_rgbd_fixture.yaml'
)


def object_cloud():
    xs = np.linspace(0.04, 0.08, 12)
    ys = np.linspace(-0.02, 0.02, 10)
    low = np.asarray([[x, y, 0.12] for x in xs for y in ys])
    high = np.asarray([[x, y, 0.18] for x in xs for y in ys])
    return np.vstack((low, high))


def table_plane():
    return TablePlane(normal=np.asarray([0.0, 0.0, 1.0]), d=-0.12, inlier_count=500)


def test_centroid_produces_a_top_grasp_from_shared_cloud():
    plan = centroid_top_grasp(
        object_cloud(), confidence=0.91, config=TopGraspConfig(),
        table_plane=table_plane(),
    )
    assert plan.backend == 'centroid'
    assert plan.method == 'sam2_rgbd_top_centroid'
    assert plan.pregrasp_m[2] > plan.grasp_m[2]
    assert plan.lift_m[2] > plan.grasp_m[2]
    assert 0.015 <= plan.grasp_width_m <= 0.085


def test_gpd_selects_gpd_geometry_without_centroid_substitution():
    mask = np.ones((100, 100), dtype=bool)
    candidate = GpdCandidate(
        score=0.8,
        width_m=0.035,
        translation_camera_m=np.asarray([0.06, 0.0, 0.18]),
        rotation_camera=np.eye(3),
    )
    plan = gpd_top_grasp(
        [candidate],
        points_target=object_cloud(),
        pixels_uv=np.zeros((1, 2)),
        mask=mask,
        intrinsics={'fx': 100.0, 'fy': 100.0, 'cx': 50.0, 'cy': 50.0},
        target_from_camera=np.eye(4),
        config=TopGraspConfig(),
        table_plane=table_plane(),
    )
    assert plan.backend == 'gpd'
    assert plan.method == 'sam2_rgbd_gpd_top'
    assert plan.score == pytest.approx(0.8)


def test_gpd_preserves_unbounded_native_ranking_score():
    mask = np.ones((100, 100), dtype=bool)
    candidates = [
        GpdCandidate(
            score=score,
            width_m=0.035,
            translation_camera_m=np.asarray([x, 0.0, 0.18]),
            rotation_camera=np.eye(3),
        )
        for score, x in ((-0.25, 0.055), (2.75, 0.06))
    ]
    plan = gpd_top_grasp(
        candidates,
        points_target=object_cloud(),
        pixels_uv=np.zeros((1, 2)),
        mask=mask,
        intrinsics={'fx': 100.0, 'fy': 100.0, 'cx': 50.0, 'cy': 50.0},
        target_from_camera=np.eye(4),
        config=TopGraspConfig(),
        table_plane=table_plane(),
    )
    assert plan.score == pytest.approx(2.75)


def test_empty_or_inconsistent_gpd_is_an_explicit_failure():
    kwargs = dict(
        points_target=object_cloud(),
        pixels_uv=np.zeros((1, 2)),
        mask=np.ones((20, 20), dtype=bool),
        intrinsics={'fx': 100.0, 'fy': 100.0, 'cx': 10.0, 'cy': 10.0},
        target_from_camera=np.eye(4),
        config=TopGraspConfig(),
        table_plane=table_plane(),
    )
    with pytest.raises(ValueError, match='no candidate'):
        gpd_top_grasp([], **kwargs)


def test_prompted_mask_decoder_rejects_wrong_shape():
    response = {
        'ok': True,
        'objects': [{
            'object_id': 0,
            'class': 'ball',
            'conf': 0.9,
            'bbox': [0, 0, 3, 2],
            'mask_b64': base64.b64encode(b'bad').decode('ascii'),
        }],
    }
    with pytest.raises(RuntimeError, match='expected'):
        decode_prompted_instance(response, width=3, height=2)


def test_masked_rgbd_cloud_and_transform_are_shared_primitives():
    depth = np.full((3, 4), 500, dtype=np.uint16)
    mask = np.zeros((3, 4), dtype=bool)
    mask[1, 2] = True
    points, pixels = masked_point_cloud(
        depth,
        mask,
        {'fx': 100.0, 'fy': 100.0, 'cx': 2.0, 'cy': 1.0,
         'depth_scale': 1000.0},
        min_depth_m=0.1,
        max_depth_m=1.0,
    )
    transform = np.eye(4)
    transform[:3, 3] = [0.1, 0.2, 0.3]
    assert pixels.tolist() == [[2, 1]]
    assert transform_cloud(points, transform)[0].tolist() == pytest.approx(
        [0.1, 0.2, 0.8]
    )


def synthetic_table_scene():
    document = yaml.safe_load(FIXTURE.read_text())
    assert document['schema'] == 'xlerobot_shared_rgbd_fixture/v1'
    height = int(document['image']['height'])
    width = int(document['image']['width'])
    depth = np.full(
        (height, width), int(document['depth']['background_mm']), dtype=np.uint16
    )
    for rectangle in document['depth']['rectangles']:
        depth[
            rectangle['y1']:rectangle['y2'], rectangle['x1']:rectangle['x2']
        ] = rectangle['depth_mm']
    sam = np.zeros((height, width), dtype=bool)
    for rectangle in document['prompted_sam_mask']['rectangles']:
        sam[
            rectangle['y1']:rectangle['y2'], rectangle['x1']:rectangle['x2']
        ] = True
    intrinsics = dict(document['camera'])
    refinement = refine_object_cloud(
        depth,
        sam,
        intrinsics,
        np.eye(4),
        min_depth_m=0.1,
        max_depth_m=1.0,
        config=RefinementConfig(
            table_min_inliers=200,
            body_cluster_min_points=20,
            sampling_stride=2,
        ),
    )
    return document, depth, intrinsics, refinement


def test_table_plane_is_removed_and_main_body_is_shared_by_both_backends():
    document, _, intrinsics, refined = synthetic_table_scene()
    assert refined.table_pixels_removed > 0
    assert refined.mask.sum() == len(refined.points_camera)
    assert refined.table_plane.normal.tolist() == pytest.approx([0.0, 0.0, 1.0])
    table_height = -refined.table_plane.d / refined.table_plane.normal[2]
    assert table_height == pytest.approx(
        document['expected']['table_height_m'], abs=0.002
    )

    config = TopGraspConfig(
        min_points=40,
        workspace_x_m=(-0.2, 0.2),
        workspace_y_m=(-0.2, 0.2),
        workspace_z_m=(0.45, 0.65),
    )
    centroid = centroid_top_grasp(
        refined.points_target,
        confidence=0.9,
        config=config,
        table_plane=refined.table_plane,
    )
    candidate = GpdCandidate(
        score=0.8,
        width_m=0.035,
        translation_camera_m=np.asarray(
            document['expected']['gpd_candidate_camera_m']
        ),
        rotation_camera=np.eye(3),
    )
    gpd = gpd_top_grasp(
        [candidate],
        points_target=refined.points_target,
        pixels_uv=refined.pixels_uv,
        mask=refined.mask,
        intrinsics=intrinsics,
        target_from_camera=np.eye(4),
        config=config,
        table_plane=refined.table_plane,
    )
    assert centroid.table_height_m == pytest.approx(table_height, abs=0.002)
    assert gpd.table_height_m == pytest.approx(table_height, abs=0.002)
    assert centroid.object_height_m > 0.05
    assert gpd.object_height_m > 0.05
    assert centroid.method == 'sam2_rgbd_top_centroid'
    assert gpd.method == 'sam2_rgbd_gpd_top'


def test_refinement_never_falls_back_to_unrefined_sam_mask():
    depth = np.full((60, 80), 500, dtype=np.uint16)
    mask = np.ones(depth.shape, dtype=bool)
    with pytest.raises(ValueError, match='no observable main 3D body'):
        refine_object_cloud(
            depth,
            mask,
            {'fx': 100.0, 'fy': 100.0, 'cx': 40.0, 'cy': 30.0,
             'depth_scale': 1000.0},
            np.eye(4),
            min_depth_m=0.1,
            max_depth_m=1.0,
            config=RefinementConfig(
                table_min_inliers=200,
                body_cluster_min_points=20,
            ),
        )


def _transform_component(model, x_semantics, y_semantics):
    transform = {
        'matrix': np.eye(4).tolist(),
        'xyz_m': [0.0, 0.0, 0.0],
        'rpy_rad': [0.0, 0.0, 0.0],
        'quaternion_xyzw': [0.0, 0.0, 0.0, 1.0],
    }
    return {
        'schema': 'xlerobot_transform_calibration/v1',
        'model': model,
        'input_sha256': '0' * 64,
        'frames': {
            'base': 'base', 'moving': 'moving',
            'camera': 'camera', 'target': 'target',
        },
        'x_semantics': x_semantics,
        'y_semantics': y_semantics,
        'x': transform,
        'y': transform,
        'metrics': {'sample_count': 12},
    }


def _runtime_files(tmp_path: Path):
    transforms = {
        'schema': 'xlerobot_runtime_transforms/v1',
        'head_camera': _transform_component(
            'moving_camera_fixed_target', 'mount_from_camera', 'base_from_target'
        ),
        'right_handeye': _transform_component(
            'fixed_camera_moving_target', 'base_from_camera', 'gripper_from_target'
        ),
    }
    alignment = {
        'schema': 'xlerobot_grasp_alignment/v1',
        'frame': 'base_link',
        'vision_fk_compensation_m': [0.005, 0.006, 0.007],
        'gravity_sag_z_m': 0.038,
        'head_pose_rad': {'pan': 0.0, 'tilt': 0.8},
        'workspace_m': {'min': [0.02, -0.09, 0.84], 'max': [0.10, -0.02, 0.88]},
        'metrics': {'sample_count': 4, 'plane_rmse_mm': 0.5},
    }
    transforms_path = tmp_path / 'transforms.yaml'
    alignment_path = tmp_path / 'grasp_alignment.yaml'
    transforms_path.write_text(yaml.safe_dump(transforms))
    alignment_path.write_text(yaml.safe_dump(alignment))
    return transforms_path, alignment_path


def test_runtime_calibration_requires_both_complete_files_and_records_provenance(tmp_path):
    transforms, alignment = _runtime_files(tmp_path)
    loaded = load_classical_runtime_calibration(str(transforms), str(alignment))
    assert loaded.vision_fk_compensation_m == pytest.approx((0.005, 0.006, 0.007))
    assert loaded.gravity_sag_z_m == pytest.approx(0.038)
    assert loaded.provenance.startswith('runtime-calibration-sha256:')

    document = yaml.safe_load(alignment.read_text())
    del document['workspace_m']
    alignment.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match='workspace_m'):
        load_classical_runtime_calibration(str(transforms), str(alignment))
