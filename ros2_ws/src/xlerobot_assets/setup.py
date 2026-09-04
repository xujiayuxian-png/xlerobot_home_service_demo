from setuptools import find_packages, setup


package_name = 'xlerobot_assets'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='xujiayuxian-png',
    maintainer_email='xujiayuxian-png@users.noreply.github.com',
    description='Versioned local artifact catalog for XLeRobot.',
    license='Apache-2.0',
)
