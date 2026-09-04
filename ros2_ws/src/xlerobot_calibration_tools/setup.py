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
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='lisa',
    maintainer_email='lisa@example.com',
    description='Offline camera and hand-eye calibration solvers.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'solve_head_camera_calibration = xlerobot_calibration_tools.cli:main_head',
            'solve_arm_handeye_calibration = xlerobot_calibration_tools.cli:main_arm',
            'collect_transform_samples = xlerobot_calibration_tools.collector_node:main',
            'detect_calibration_target = xlerobot_calibration_tools.target_node:main',
        ],
    },
)
