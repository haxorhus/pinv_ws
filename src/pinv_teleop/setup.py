from setuptools import find_packages, setup

package_name = 'pinv_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},   # sin esto colcon test no corre pytest
    zip_safe=True,
    maintainer='jose',
    maintainer_email='jose@todo.todo',
    description='Control manual del ASV PINV desde el teclado.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'teleop_teclado = pinv_teleop.teleop_teclado:main',
        ],
    },
)
