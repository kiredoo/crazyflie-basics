from setuptools import find_packages
from setuptools import setup

setup(
    name='crazyflie_interfaces',
    version='2.0.0',
    packages=find_packages(
        include=('crazyflie_interfaces', 'crazyflie_interfaces.*')),
)
