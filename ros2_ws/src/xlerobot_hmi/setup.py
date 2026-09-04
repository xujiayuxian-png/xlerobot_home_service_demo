from pathlib import Path

from setuptools import find_packages, setup


package_name = 'xlerobot_hmi'
web_dist = Path('web/dist')
web_files = [
    (
        f'share/{package_name}/web_dist/{path.parent.relative_to(web_dist)}',
        [str(path)],
    )
    for path in web_dist.glob('**/*')
    if path.is_file()
]

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
    ] + web_files,
    install_requires=['setuptools', 'aiohttp', 'PyYAML'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='xujiayuxian-png',
    maintainer_email='xujiayuxian-png@users.noreply.github.com',
    description='Product Operator Console and ROS web gateway.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fetch_deliver = xlerobot_hmi.fetch_deliver_cli:main',
            'operator_console = xlerobot_hmi.operator_console:main',
        ],
    },
)
