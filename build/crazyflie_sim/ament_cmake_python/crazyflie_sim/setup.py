from setuptools import find_packages
from setuptools import setup

setup(
    name='crazyflie_sim',
    version='2.0.0',
    packages=find_packages(
        include=('crazyflie_sim', 'crazyflie_sim.*')),
)
