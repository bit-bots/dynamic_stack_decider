# -*- coding:utf-8 -*-
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['dynamic_stack_decider'],
    #scripts=['bin/myscript'],
    package_dir={'': 'src'}
)

setup(**d)
