"""Launch only the base geometry calibration tool."""

from xlerobot_bringup.calibration_launch import calibration_launch


def generate_launch_description():
    return calibration_launch('base_geometry')
