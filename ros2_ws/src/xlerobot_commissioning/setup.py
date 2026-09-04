from setuptools import find_packages, setup


package_name = 'xlerobot_commissioning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='lisa',
    maintainer_email='lisa@example.com',
    description='Commissioning workflow coordinators for XLeRobot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'site_manager = xlerobot_commissioning.site_manager_node:main',
            'calibration_workbench = xlerobot_commissioning.calibration_node:main',
            'collect_episode = xlerobot_commissioning.collection_node:main',
        ],
    },
)
