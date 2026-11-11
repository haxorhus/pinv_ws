from glob import glob
import os

from setuptools import setup

package_name = 'pinv_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jose',
    maintainer_email='jose@todo.todo',
    description='Lanzadores del ASV PINV (bote real y agente micro-ROS).',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [],
    },
)
