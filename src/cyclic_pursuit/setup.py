from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cyclic_pursuit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kiredoo',
    maintainer_email='chl146@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = cyclic_pursuit.controller:main',
            'pose_vir = cyclic_pursuit.variable_pose_publisher:main',
            'landing_client = cyclic_pursuit.landing:main',
        ],
    },
)
