import json
from pathlib import Path

import pytest

from xlerobot_assets import ArtifactCatalog


def test_create_activate_export_and_reindex(tmp_path: Path):
    catalog = ArtifactCatalog(tmp_path)
    reference = catalog.create(
        kind='sites',
        artifact_id='home',
        version='v1',
        schema='xlerobot_site/v1',
        files={'map.yaml': 'image: map.pgm\n', 'map.pgm': b'P5\n1 1\n255\n\x00'},
        metadata={'software_revision': 'abc123'},
    )
    manifest = json.loads((reference.path / 'manifest.json').read_text())
    assert manifest['schema'] == 'xlerobot_site/v1'
    assert set(manifest['checksums']) == {'map.pgm', 'map.yaml'}
    assert catalog.activate('sites', 'home', 'v1') == reference
    assert catalog.active('sites', 'home') == reference
    assert (tmp_path / 'sites/home/current').resolve() == reference.path.resolve()
    second = catalog.create(
        kind='sites', artifact_id='home', version='v2',
        schema='xlerobot_site/v1', files={'map.yaml': 'image: map.png\n'},
    )
    catalog.activate('sites', 'home', 'v2')
    assert (tmp_path / 'sites/home/current').resolve() == second.path.resolve()
    catalog.activate('sites', 'home', 'v1')
    assert (tmp_path / 'sites/home/current').resolve() == reference.path.resolve()
    assert catalog.export(reference).is_file()
    assert catalog.versions('sites', 'home') == [reference, second]
    assert catalog.rebuild_index() == 2


def test_artifacts_are_immutable_and_paths_are_bounded(tmp_path: Path):
    catalog = ArtifactCatalog(tmp_path)
    catalog.create(
        kind='units', artifact_id='robot1', version='v1',
        schema='xlerobot_unit_calibration/v1', files={'calibration.yaml': '{}\n'},
    )
    with pytest.raises(FileExistsError):
        catalog.create(
            kind='units', artifact_id='robot1', version='v1',
            schema='xlerobot_unit_calibration/v1', files={'other': 'x'},
        )
    with pytest.raises(ValueError):
        catalog.create(
            kind='sites', artifact_id='home', version='v2',
            schema='xlerobot_site/v1', files={'../escape': 'x'},
        )
