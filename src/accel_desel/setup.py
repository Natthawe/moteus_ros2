import os
from glob import glob
from setuptools import setup

package_name = 'accel_desel'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gmc_cg',
    maintainer_email='gmc_cg@todo.todo',
    description='accel_desel',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'accel_desel = accel_desel.accel_desel:main'
        ],
    },
)
