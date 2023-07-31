import os
from glob import glob

from setuptools import setup

package_name = 'pokebo_cube'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='icd',
    maintainer_email='sekikawa.taichi.vf@tut.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cube_core = pokebo_cube.cube_core:main',
            'utterance_core = pokebo_cube.utterance_core:main',
        ],
    },
)
