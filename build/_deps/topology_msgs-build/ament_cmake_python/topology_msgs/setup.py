from setuptools import find_packages
from setuptools import setup

setup(
    name='topology_msgs',
    version='0.0.1',
    packages=find_packages(
        include=('topology_msgs', 'topology_msgs.*')),
)
