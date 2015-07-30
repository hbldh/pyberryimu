# -*- coding: utf-8 -*-
"""
:mod:`setup.py` -- PyBerryIMU Setup file
======================================

.. module:: setup
   :platform: Unix, Windows
   :synopsis: The Python Packaging setup file for PyBerryIMU.

.. moduleauthor:: hbldh <henrik.blidh@nedomkull.com>

Created on 2013-09-14, 19:31

"""

from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

import pyberryimu
from setuptools import setup, find_packages

setup(
    name='pyberryimu',
    version=pyberryimu.__version__,
    author=pyberryimu.author,
    author_email=pyberryimu.author_email,
    maintainer=pyberryimu.maintainer,
    maintainer_email=pyberryimu.maintainer_email,
    url=pyberryimu.url,
    download_url=pyberryimu.download_url,
    description=pyberryimu.description,
    long_description=pyberryimu.long_description,
    license=pyberryimu.license,
    platforms=pyberryimu.platforms,
    keywords=pyberryimu.keywords,
    classifiers=pyberryimu.classifiers,
    packages=find_packages(),
    package_data={
        'docs': [
            '*',
        ],
    },
    install_requires=[line.strip() for line in open("requirements.txt")],
    dependency_links=[],
    ext_modules=[],
    entry_points={
        'console_scripts': [
        ]
    }
)

