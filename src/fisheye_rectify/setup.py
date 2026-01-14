from setuptools import find_packages, setup

package_name = 'fisheye_rectify'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'fisheye_rectify_node = fisheye_rectify.fisheye_rectify_node:main',
		'click_points = fisheye_rectify.click_points:main',
		'birdseye_node = fisheye_rectify.birdseye_node:main',
        ],
    },
)
