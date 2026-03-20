from setuptools import find_packages
from setuptools import setup

setup(
    name='spin_conversion',
    version='1.3.1',
    packages=find_packages(
        include=('spin_conversion', 'spin_conversion.*')),
)
