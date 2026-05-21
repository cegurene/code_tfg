from setuptools import find_packages
from setuptools import setup

setup(
    name='rl_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('rl_interfaces', 'rl_interfaces.*')),
)
