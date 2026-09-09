from setuptools import find_packages, setup


package_name = 'xlerobot_calibration_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [f'resource/{package_name}'],
        ),
        (f'share/{package_name}', ['package.xml']),
        (
            f'share/{package_name}/config',
            [
                'config/d455_head_calibration.yaml',
                'config/head_camera_poses.yaml',
                'config/quality.yaml',
                'config/right_handeye_poses.yaml',
                'config/targets.yaml',
            ],
        ),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='xujiayuxian-png',
    maintainer_email='xujiayuxian-png@users.noreply.github.com',
    description='Offline camera and hand-eye calibration solvers.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hover_validation = xlerobot_calibration_tools.hover_node:main',
            'solve_head_camera_calibration = xlerobot_calibration_tools.cli:main_head',
            'solve_arm_handeye_calibration = xlerobot_calibration_tools.cli:main_arm',
            'collect_transform_samples = xlerobot_calibration_tools.collector_node:main',
            'detect_calibration_target = xlerobot_calibration_tools.target_node:main',
            'auto_head_calibration = xlerobot_calibration_tools.head_auto_node:main',
            'auto_handeye_calibration = xlerobot_calibration_tools.head_auto_node:main_handeye',
            'xlerobot-calibrate = xlerobot_calibration_tools.public_cli:main',
        ],
    },
)
