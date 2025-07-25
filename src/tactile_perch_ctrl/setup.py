from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tactile_perch_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add this line for launch files
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antbre',
    maintainer_email='a.bredenbeck@tudelft.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'touch_sensor_driver = tactile_perch_ctrl.touch_sensor_driver:main',
            'feely_drone_state_machine = tactile_perch_ctrl.state_machine_node:main',
            'simple_demo_state_machine = tactile_perch_ctrl.simple_demo_state_machine_node:main',
        ],
    },
)
