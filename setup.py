#!/usr/bin/env python

from pathlib import Path


with open(f'{Path(__file__).parent}/requirements.txt', 'r') as f:
   pip_dependencies = f.readlines()


# Try catkin install
try:
   from catkin_pkg.python_setup import generate_distutils_setup
   from distutils.core import setup

   d = generate_distutils_setup(
      packages=['rl_tasks'],
      package_dir={'': 'src'}
   )

   setup(**d)

# Switch to regular pip install
except ModuleNotFoundError:
   from setuptools import setup, find_packages

   setup(
      name='RL Sim Tasks',
      version='0.0.1',
      author='Adrian Roefer',
      author_email='aroefer@cs.uni-freiburg.de',
      packages=['rl_tasks'],
      package_dir={'': 'src'},
      # scripts=['bin/script1','bin/script2'],
      url='http://pypi.python.org/pypi/rl_tasks/',
      license='LICENSE',
      description='A collection of object models and micro scenarios for doing '
                  'mobile manipulation in the Freiburg RL kitchen environment.',
      # long_description=open('README.txt').read(),
      install_requires=pip_dependencies,
   )
