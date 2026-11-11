from setuptools import setup
from glob import glob
import os

package_name = 'jeepbot_sim'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JeepBot team',
    maintainer_email='maintainer@example.invalid',
    description='Four-wheel JeepBot Gazebo skid-steer and virtual VESC simulator',
    license='MIT',
    entry_points={'console_scripts': [
        'virtual_vescs = jeepbot_sim.virtual_vescs:main',
        'sim_skid_controller = jeepbot_sim.skid_controller:main',
    ]},
)
