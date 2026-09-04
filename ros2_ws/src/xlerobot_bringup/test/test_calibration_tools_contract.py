from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_four_calibration_entrypoints_select_one_shared_workflow_each():
    expected = {
        'servo_calibration.launch.py': 'servo',
        'base_geometry_calibration.launch.py': 'base_geometry',
        'd455_extrinsic_calibration.launch.py': 'head_camera',
        'right_handeye_calibration.launch.py': 'right_handeye',
    }
    for filename, workflow in expected.items():
        source = (ROOT / 'launch' / filename).read_text()
        assert f"calibration_launch('{workflow}')" in source
        assert 'controller_manager' not in source
        assert 'imu' not in source.lower()
    assert not (ROOT / 'launch/calibration_workbench.launch.py').exists()


def test_shared_composition_passes_fixed_workflow_to_backend_and_console():
    source = (ROOT / 'xlerobot_bringup/calibration_launch.py').read_text()
    assert "'workflow_id': workflow_id" in source
    assert "'calibration_workflow': workflow_id" in source
    assert "'workspace': 'calibration'" in source
    assert "executable='calibration_workbench'" in source
    assert "workflow_id in {'head_camera', 'right_handeye'}" in source
    assert "'platform_runtime.launch.py'" in source
    assert "'enable_d455': 'true'" in source
    assert "'right_bus': LaunchConfiguration('right_bus')" in source
    assert "'left_bus': LaunchConfiguration('left_bus')" in source
    assert "'d455_serial': LaunchConfiguration('d455_serial')" in source
    assert "executable='detect_calibration_target'" in source
    assert "executable='collect_transform_samples'" in source
    assert "executable='calibration_pose_server'" in source
    assert "'execution_enabled': True" in source
    assert "'head_camera_link' if workflow_id == 'head_camera'" in source
    assert "'startup_ready': 'false'" in source
    assert "workflow_id == 'servo'" in source
    assert "executable='servo_calibration_server'" in source
    assert 'technician_pin' not in source
