from pathlib import Path


SOURCE = Path(__file__).parents[1] / 'src/servo_calibration_server.cpp'


def test_servo_calibrator_can_only_read_positions_and_release_torque():
    source = SOURCE.read_text()
    assert 'readPosition' in source
    assert 'enableTorque(item->id, false)' in source
    assert 'syncWritePosition' not in source
    assert 'syncWriteVelocity' not in source
    assert 'initPositionMotor' not in source


def test_servo_calibrator_requires_all_fourteen_joints_before_finalize():
    source = SOURCE.read_text()
    table = source[source.index('const std::vector<JointSpec> kJoints'):source.index(
        'class ServoCalibrationServer'
    )]
    assert table.count('{"right_arm",') == 6
    assert table.count('{"left_arm",') == 6
    assert table.count('{"head",') == 2
    assert 'calibration is incomplete:' in source
    assert 'recorded range covers less than 60%' in source
