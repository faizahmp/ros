from setuptools import setup
from glob import glob
import os

package_name = 'final_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.states', f'{package_name}.yolo_inference'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu',
    maintainer_email='tedev@email.com',
    description='Turtlebot patrol and object detection with state machine',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sm_main = final_pkg.sm_main:main',
        ],
    },
    
)
