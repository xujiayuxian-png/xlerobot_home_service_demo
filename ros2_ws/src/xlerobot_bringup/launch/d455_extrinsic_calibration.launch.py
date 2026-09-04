"""Launch only the head-mounted D455 extrinsic calibration tool."""

from xlerobot_bringup.calibration_launch import calibration_launch


def generate_launch_description():
    return calibration_launch('head_camera')
