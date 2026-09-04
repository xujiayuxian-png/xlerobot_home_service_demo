"""Launch only the right-arm hand-eye calibration tool."""

from xlerobot_bringup.calibration_launch import calibration_launch


def generate_launch_description():
    return calibration_launch('right_handeye')
