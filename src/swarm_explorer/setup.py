"""
Setup of Swarm Explorer python codebase
"""
from setuptools import setup

setup(name='swarm_explorer',
      version='0.1.0',
      description='Swarm Explorer Final Project for EECS106B',
      package_dir = {'': 'src'},
      packages=['swarm_explorer'],
      install_requires=[],
      test_suite='test'
     )
