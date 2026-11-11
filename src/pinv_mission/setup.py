from setuptools import find_packages, setup

package_name = 'pinv_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jose',
    maintainer_email='jose@todo.todo',
    description='Misiones del ASV PINV: waypoints, regreso a casa y evaluación.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [],
    },
)
