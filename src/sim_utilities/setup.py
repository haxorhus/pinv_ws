import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sim_utilities'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jose',
    maintainer_email='jose@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_node = sim_utilities.cmd_node:main',
            'cmd_translator_node = sim_utilities.cmd_translator:main',
            'empty_map_pub = sim_utilities.empty_map_pub:main',
            'republisher = sim_utilities.republisher_node:main',
            'transform_pub = sim_utilities.transform_pub:main'
        ],
    },
)
