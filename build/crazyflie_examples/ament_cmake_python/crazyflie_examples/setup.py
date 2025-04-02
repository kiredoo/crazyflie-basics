from setuptools import find_packages
from setuptools import setup

setup(
    name='crazyflie_examples',
    version='2.0.0',
    packages=find_packages(
        include=('crazyflie_examples', 'crazyflie_examples.*')),
)
