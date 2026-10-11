#!/usr/bin/env python3
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(
    packages=['morai_sensor_fusion', 'morai_sensor_fusion.ufld_vendor'],
    package_dir={'': 'src'},
))
