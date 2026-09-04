from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'xlerobot_perception'

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
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='xujiayuxian-png',
    maintainer_email='xujiayuxian-png@users.noreply.github.com',
    description='RGB-D object localization and grasp planning for the XLeRobot demo.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'detect_object_vlm = xlerobot_perception.detect_object_node:main',
            'verify_grasp_vlm = xlerobot_perception.verify_grasp_node:main',
            'scan_for_person_detector = xlerobot_perception.scan_for_person_node:main',
        ],
    },
)
