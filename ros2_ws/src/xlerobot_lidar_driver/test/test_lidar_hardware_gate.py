import os
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix


def lidar_executable():
    return os.path.join(
        get_package_prefix('xlerobot_lidar_driver'),
        'lib',
        'xlerobot_lidar_driver',
        'lidar_node',
    )


def test_closed_hardware_gate_refuses_device_access():
    result = subprocess.run(
        [
            lidar_executable(),
            '--ros-args',
            '-p',
            'mock_hardware:=false',
            '-p',
            'hardware_enabled:=false',
            '-p',
            'port_name:=/dev/xlerobot_gate_must_not_open',
        ],
        capture_output=True,
        text=True,
        timeout=5,
        check=False,
    )
    output = result.stdout + result.stderr
    assert result.returncode == 1
    assert 'hardware gate is closed' in output


def test_mock_mode_stays_alive_with_nonexistent_device_path():
    process = subprocess.Popen(
        [
            lidar_executable(),
            '--ros-args',
            '-p',
            'mock_hardware:=true',
            '-p',
            'hardware_enabled:=false',
            '-p',
            'port_name:=/dev/xlerobot_mock_must_not_open',
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        time.sleep(0.8)
        assert process.poll() is None
        process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=5)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)
    assert process.returncode == 0
    assert 'deterministic mock mode' in output
