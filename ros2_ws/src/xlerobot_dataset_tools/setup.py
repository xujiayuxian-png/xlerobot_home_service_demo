from setuptools import find_packages, setup


package_name = 'xlerobot_dataset_tools'

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
        (f'share/{package_name}/config', ['config/two_wheel_pick.yaml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='lisa',
    maintainer_email='lisa@example.com',
    description='NPZ and dual-MP4 ACT episode capture and dataset tools.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'record_episode_node = xlerobot_dataset_tools.record_episode_node:main',
            'episode_review_node = xlerobot_dataset_tools.review_node:main',
        ],
    },
)
