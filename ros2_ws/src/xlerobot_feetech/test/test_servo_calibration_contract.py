from pathlib import Path


SOURCE = Path(__file__).parents[1] / 'src/servo_calibration_server.cpp'
SESSION = Path(__file__).parents[1] / 'src/servo_calibration_session.cpp'


def test_servo_calibrator_can_only_read_positions_and_release_torque():
    source = SOURCE.read_text()
    assert 'syncReadPositions' in source
    assert 'enableTorque(item.id, false)' in source
    assert 'syncWritePosition' not in source
    assert 'syncWriteVelocity' not in source
    assert 'initPositionMotor' not in source
    assert 'initVelocityMotor' not in source
    assert 'create_wall_timer(50ms' in source
    assert 'request->joint' not in source


def test_servo_calibrator_requires_all_fourteen_joints_before_finalize():
    source = SESSION.read_text()
    table = source[source.index('static const std::vector<ServoCalibrationSpec> rows'):source.index(
        'return rows;'
    )]
    assert table.count('{"right_arm",') == 6
    assert table.count('{"left_arm",') == 6
    assert table.count('{"head",') == 2
    assert 'calibration is incomplete:' in source
    assert 'kMinimumCoverage = 0.60' in source


def test_every_response_has_all_rows_and_web_polling_does_not_write_hardware():
    source = SOURCE.read_text()
    status = source[source.index('request->command == Service::Request::STATUS'):source.index(
        'request->command == Service::Request::SCAN'
    )]
    assert 'refresh(' not in status
    assert 'enableTorque' not in status
    assert 'response.joints.push_back' in source
    assert 'response_state(*response);' in source
    assert 'std::set<std::string> released_groups_' in source
