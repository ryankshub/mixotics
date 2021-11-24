#!/usr/bin/env python3
# Setup file to access python packages
# RKS
# Note: this file is sourced from 
# https://nu-msr.github.io/me495_site/lecture02_catkin.html#orgabccbfa
# Original author: Matt Elwin

# Project imports

# Python imports
from setuptools import setup

# 3rd party imports
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['menu_utilities'],
    package_dir={'': 'src'}
    )
setup(**d)