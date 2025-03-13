from setuptools import find_packages
from setuptools import setup

setup(
    name='leader_bot',
    version='0.0.1',
    packages=find_packages(
        include=('leader_bot', 'leader_bot.*')),
)
