#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
:mod:`datasheet`
==================

.. module:: datasheet
   :platform: Unix, Windows
   :synopsis: 

.. moduleauthor:: hbldh <henrik.blidh@nedomkull.com>

Created on 2015-07-30, 00:27

"""

from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import

import os
from pyberryimu import version


class DataSheetCalibration(object):
    """Converting using values from data sheets.

    Read more about these values in the
    `LSM9DS0 data sheet <http://ozzmaker.com/wp-content/uploads/2014/12/LSM9DS0.pdf>`_.

    """

    def __init__(self, verbose=True):
        """Constructor for DataSheetCalibration"""
        self._verbose = verbose

        # BerryIMU settings for the client used for calibration.
        self.pyberryimu_version = version
        self.berryimu_settings = None

        self._acc_unit_per_lsb = None
        self._gyro_unit_per_lsb = None
        self._mag_unit_per_lsb = None

    def to_json(self):
        return {
            'pyberryimu_version': version,
            'BerryIMU_settings': self.berryimu_settings,
        }

    def calibrate_accelerometer(self, client, **kwargs):
        """Perform calibration of the BerryIMU accelerometer."""
        self._acc_unit_per_lsb = {
            2: 0.061 / 1000.,
            4: 0.122 / 1000.,
            6: 0.183 / 1000.,
            8: 0.244 / 1000.,
            16: 0.732 / 1000.
        }.get(client.get_settings().get('accelerometer').get('full_scale'))

    def calibrate_gyroscope(self, client, **kwargs):
        """Perform calibration of the BerryIMU gyroscope."""
        self._gyro_unit_per_lsb = {
            245:  8.75 / 1000.,
            500: 17.50 / 1000.,
            2000: 70 / 1000.,
        }.get(client.get_settings().get('gyroscope').get('full_scale'))

    def calibrate_magnetometer(self, client, **kwargs):
        """Perform calibration of the BerryIMU magnetometer."""
        self._mag_unit_per_lsb = {
            2: 0.08 / 1000.,
            4: 0.16 / 1000.,
            8: 0.32 / 1000.,
            12: 0.48 / 1000.
        }.get(client.get_settings().get('magnetometer').get('full_scale'))

    def transform_accelerometer_values(self, acc_values):
        return tuple([(a / self._acc_unit_per_lsb) for a in acc_values])

    def transform_gyroscope_values(self, gyro_values):
        return tuple([(g / self._gyro_unit_per_lsb) for g in gyro_values])

    def transform_magnetometer_values(self, mag_values):
        return tuple([(m / self._mag_unit_per_lsb) for m in mag_values])

