from glob import glob

from setuptools import find_packages, setup


package_name = 'xlerobot_policy'

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
        (f'share/{package_name}/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='lisa',
    maintainer_email='lisa@example.com',
    description='Untrusted policy clients and deterministic postprocessing.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'act_policy_adapter = xlerobot_policy.act_policy_node:main',
            'policy_capture_sink = xlerobot_policy.policy_capture_sink:main',
        ],
    },
)
