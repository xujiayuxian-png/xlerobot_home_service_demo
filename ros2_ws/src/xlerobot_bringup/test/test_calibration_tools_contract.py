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
    assert "'hardware_enabled', default_value='false'" in source
    assert 'OpaqueFunction(function=_require_explicit_hardware)' in source
    assert "'calibration_capture_only': True" in source
    assert "'capture_only': True" in source
    assert "'capture_result_file': result_file" in source
    assert "'task_history_root': LaunchConfiguration('task_history_root')" in source
    assert "'workflow_id': workflow_id" in source
    assert "'calibration_workflow': workflow_id" in source
    assert "'workspace': 'calibration'" in source
    assert "executable='calibration_workbench'" in source
    assert "workflow_id in {'head_camera', 'right_handeye'}" in source
    assert "'platform_runtime.launch.py'" in source
    assert "'hardware_enabled': 'true'" in source
    assert "'enable_d455': 'true'" in source
    assert "'right_bus': LaunchConfiguration('right_bus')" in source
    assert "'left_bus': LaunchConfiguration('left_bus')" in source
    assert "'d455_serial': LaunchConfiguration('d455_serial')" in source
    assert "'d455_config_file': PathJoinSubstitution([" in source
    assert "'d455_head_calibration.yaml'" in source
    assert "executable='detect_calibration_target'" in source
    assert "executable='collect_transform_samples'" in source
    assert "executable='calibration_pose_server'" in source
    assert "'execution_enabled': True" in source
    assert "'head_camera_link' if workflow_id == 'head_camera'" in source
    assert "'startup_ready': 'false'" in source
    assert "workflow_id == 'servo'" in source
    assert "executable='servo_calibration_server'" in source
    assert 'technician_pin' not in source
    assert "'existing_servo_file': LaunchConfiguration('existing_servo_file')" in source
    assert "'existing_servo_version': LaunchConfiguration('existing_servo_version')" in source
    assert "'unit_id': LaunchConfiguration('unit_id')" in source


def test_head_camera_reuses_head_only_runtime_and_public_auto_pipeline():
    source = (ROOT / 'xlerobot_bringup/calibration_launch.py').read_text()
    assert "'head_only_control': 'true' if workflow_id == 'head_camera' else 'false'" in source
    assert "executable = 'auto_head_calibration' if workflow_id == 'head_camera' else 'auto_handeye_calibration'" in source
    assert "'right_handeye_control': 'true' if workflow_id == 'right_handeye' else 'false'" in source
    assert "'head_pose_file': PathJoinSubstitution([" in source
    assert "'pose_file': PathJoinSubstitution([" in source
    assert "f'{workflow_id}_poses.yaml'" in source
    assert "'right_handeye_poses.yaml'" in source
    assert "'state_root': LaunchConfiguration('state_root')" in source
    assert "'repo_root': LaunchConfiguration('repo_root')" in source


def test_public_capture_wrapper_requires_explicit_hardware_before_ros():
    wrapper = ROOT.parents[2] / 'tools/calibrate'
    completed = __import__('subprocess').run(
        [str(wrapper), 'capture', 'servo'], text=True, capture_output=True,
        check=False,
    )
    assert completed.returncode == 2
    assert 'no device was opened' in completed.stderr

    helper = (ROOT.parents[2] / 'tools/lib/calibration_capture.sh').read_text()
    assert "printf -v config_q '%q' \"$XLEROBOT_CONFIG\"" in helper
    assert helper.count('--config $config_q') == 3
    assert 'render --for "$workflow"' in helper
    assert '--fresh and --resume are mutually exclusive' in helper
    assert 'previous capture archived without deletion' in helper
    assert 'verify_calibration_runtime >/dev/null' in helper
    assert 'not applied automatically' in helper


def test_servo_capture_rejects_unit_mismatch_before_opening_devices(tmp_path):
    import subprocess
    import yaml

    config = tmp_path / 'local.yaml'
    config.write_text(yaml.safe_dump({
        'schema': 'xlerobot_demo/v1',
        'robot': {'unit_id': 'robot-a'},
        'calibration': {'unit': 'robot-b', 'state_root': str(tmp_path / 'state')},
    }))
    result = subprocess.run([
        str(ROOT.parents[2] / 'tools/calibrate'), 'capture', 'servo',
        '--hardware', '--config', str(config),
    ], capture_output=True, text=True, check=False)
    assert result.returncode == 2
    assert 'robot.unit_id and calibration.unit must be the same' in result.stderr
    assert not (tmp_path / 'state').exists()
