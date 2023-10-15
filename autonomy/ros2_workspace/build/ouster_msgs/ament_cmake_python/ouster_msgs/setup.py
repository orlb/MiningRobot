from setuptools import find_packages
from setuptools import setup

setup(
    name='ouster_msgs',
    version='0.5.0',
    packages=find_packages(
        include=('ouster_msgs', 'ouster_msgs.*')),
)
