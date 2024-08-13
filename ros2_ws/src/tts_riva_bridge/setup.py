import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'tts_riva_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'nvidia-riva-client'],
    zip_safe=True,
    maintainer='robin',
    maintainer_email='robin.karlsson0@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tts_riva_bridge = tts_riva_bridge.tts_riva_bridge:main',
        ],
    },
)
