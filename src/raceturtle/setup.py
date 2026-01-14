from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'raceturtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amrl-linux24-04',
    maintainer_email='ben.schuhknecht@gmx.net',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'birdseye_view = raceturtle.birdseye_view:main'
        ],
    },
)
