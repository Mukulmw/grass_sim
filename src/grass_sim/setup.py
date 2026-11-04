from glob import glob

from setuptools import find_packages, setup

package_name = 'grass_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mukul',
    maintainer_email='mukulmw@gmail.com',
    description='RViz simulation of the GRASS robot',
    license='TODO',
    entry_points={
        'console_scripts': [
            'sim_arduino = grass_sim.sim_arduino_node:main',
            'scene = grass_sim.scene_node:main',
            'kin_check = grass_sim.kin_check_node:main',
        ],
    },
)
