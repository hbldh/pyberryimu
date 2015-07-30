#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
:mod:`base`
==================

.. module:: base
   :platform: Unix, Windows
   :synopsis: 

.. moduleauthor:: hbldh <henrik.blidh@nedomkull.com>

Created on 2015-05-28, 13:50

"""

from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import

import os
from pyberryimu import version


class BerryIMUCalibration(object):
    """Default object for calibrators.

    Returns raw integer output from the BerryIMU.

    """

    def __init__(self, verbose=True):
        """Constructor for BerryIMUCalibration"""
        self._verbose = verbose

        # BerryIMU settings for the client used for calibration.
        self.pyberryimu_version = version
        self.berryimu_settings = None

    def __str__(self):
        return "BerryIMUCalibration (Raw data returned)"

    def __repr__(self):
        return str(self)

    @classmethod
    def load(cls, doc_path=os.path.expanduser('~/.pyberryimu')):
        raise NotImplementedError("Base BerryIMUCalibration is non-loadable.")

    def save(self, save_path=os.path.expanduser('~/.pyberryimu')):
        raise NotImplementedError("Base BerryIMUCalibration is non-savable.")

    def to_json(self):
        return {
            'pyberryimu_version': version,
            'BerryIMU_settings': self.berryimu_settings,
        }

    def calibrate_accelerometer(self, client, **kwargs):
        """Perform calibration of the BerryIMU accelerometer."""
        raise NotImplementedError("Base BerryIMUCalibration is not possible to calibrate.")

    def calibrate_gyroscope(self, client, **kwargs):
        """Perform calibration of the BerryIMU gyroscope."""
        raise NotImplementedError("Base BerryIMUCalibration is not possible to calibrate.")

    def calibrate_magnetometer(self, client, **kwargs):
        """Perform calibration of the BerryIMU magnetometer."""
        raise NotImplementedError("Base BerryIMUCalibration is not possible to calibrate.")

    def transform_accelerometer_values(self, acc_values):
        return acc_values

    def transform_gyroscope_values(self, gyro_values):
        return gyro_values

    def transform_magnetometer_values(self, mag_values):
        return mag_values

