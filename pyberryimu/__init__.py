# -*- coding: utf-8 -*-
"""Release data for the PyBerryIMU project."""

# -----------------------------------------------------------------------------
# Copyright (c) 2015, Nedomkull Mathematical Modeling AB.
# -----------------------------------------------------------------------------


author = 'Henrik Blidh'
author_email = 'henrik.blidh@nedomkull.com'
maintainer = 'Henrik Blidh'
maintainer_email = 'henrik.blidh@nedomkull.com'
license = 'MIT'
description = "PyBerryIMU - I2C communications lib for using BerryIMU"
long_description = \
    """
    """
url = 'https://github.com/hbldh/pyberryimu'
download_url = 'https://github.com/hbldh/pyberryimu/downloads'
platforms = ['Linux']
keywords = []
classifiers = [
    'Programming Language :: Python :: 2.6',
    'Programming Language :: Python :: 2.7',
    'Programming Language :: Python :: 3.3',
    'Programming Language :: Python :: 3.4',
    'Operating System :: POSIX :: Linux',
    'Development Status :: 4 - Beta',
    'Intended Audience :: Developers',
    'Topic :: Scientific/Engineering :: Mathematics',
]

# Version information.  An empty _version_extra corresponds to a full
# release.  'dev' as a _version_extra string means this is a development
# version.
_version_major = 0
_version_minor = 4
_version_patch = 2
#_version_extra = 'dev3'
#_version_extra = 'a1'
_version_extra = ''  # Uncomment this for full releases

# Construct full version string from these.
_ver = [_version_major, _version_minor, _version_patch]

__version__ = '.'.join(map(str, _ver))
if _version_extra:
    __version__ += _version_extra

version = __version__  # backwards compatibility name
version_info = (_version_major, _version_minor, _version_patch, _version_extra)
