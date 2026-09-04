from pathlib import Path

from setuptools import find_packages, setup


package_name = 'xlerobot_voice'
root = Path(__file__).parent


def package_files(pattern):
    return [str(path) for path in root.glob(pattern)]


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
        (f'share/{package_name}/config', package_files('config/*.yaml')),
        (f'share/{package_name}/config', package_files('config/*.txt')),
        (f'share/{package_name}/audio', package_files('audio/*.mp3')),
        (f'share/{package_name}/launch', package_files('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='xujiayuxian-png',
    maintainer_email='xujiayuxian-png@users.noreply.github.com',
    description='Gated local voice adapter for typed home-service tasks.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'speak_text = xlerobot_voice.speak_text_node:main',
            'voice_assistant = xlerobot_voice.voice_assistant_node:main',
        ],
    },
)
