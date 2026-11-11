from glob import glob
import os

from setuptools import setup

package_name = 'pinv_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jose',
    maintainer_email='jose@todo.todo',
    description='Localización del ASV PINV: EKF con IMU + GPS RTK.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [],
    },
)
