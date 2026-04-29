from setuptools import setup

package_name = 'drone_control'

setup(
name=package_name,
version='0.0.1',
packages=[package_name],
install_requires=['setuptools'],
zip_safe=True,
maintainer='you',
maintainer_email='[you@email.com](mailto:you@email.com)',
description='Drone control nodes',
license='Apache-2.0',
entry_points={
'console_scripts': [
'takeoff_node = drone_control.takeoff_node:main',
'lidar_processor = drone_control.lidar_processor:main',
'avoidance_node = drone_control.avoidance_node:main',
'keyboard_control = drone_control.keyboard_control:main',  # ✅ ADDED
],
},
)
