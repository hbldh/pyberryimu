#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
:mod:`container`
==================

.. module:: container
   :platform: Unix, Windows
   :synopsis: 

.. moduleauthor:: hbldh <henrik.blidh@nedomkull.com>

Created on 2015-06-17, 13:58

"""

from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import

import os
import json
import datetime

import numpy as np

from pyberryimu import version


class IMUDataContainer(object):

    def __init__(self, start_time, client_settings, calibration_parameters=None):

        self.recording_name = None
        self.version = {'pyberryimu': version}

        self.start_time = start_time
        self.client_settings = client_settings
        self.calibration_parameters = calibration_parameters
        self._data = {
            'timestamps': None,
            'accelerometer': None,
            'gyroscope': None,
            'magnetometer': None,
            'pressure': None,
            'temperature': None
        }

    def __str__(self):
        "{0}, {1} samples".format(self.start_time.strftime('%Y-%m-%d %H:%M:%S'), len(self))

    def __len__(self):
        return len(self.timestamps) if self.timestamps is not None else 0

    @property
    def timestamps(self):
        return self._data.get('timestamps')

    @timestamps.setter
    def timestamps(self, value):
        if value is not None:
            self._data['timestamps'] = np.array(value)

    @property
    def accelerometer(self):
        return self._data.get('accelerometer')

    @accelerometer.setter
    def accelerometer(self, value):
        if value is not None:
            self._data['accelerometer'] = np.array(value)

    @property
    def gyroscope(self):
        return self._data.get('gyroscope')

    @gyroscope.setter
    def gyroscope(self, value):
        if value is not None:
            self._data['gyroscope'] = np.array(value)

    @property
    def magnetometer(self):
        return self._data.get('magnetometer')

    @magnetometer.setter
    def magnetometer(self, value):
        if value is not None:
            self._data['magnetometer'] = np.array(value)

    @property
    def pressure(self):
        return self._data.get('pressure')

    @pressure.setter
    def pressure(self, value):
        if value is not None:
            self._data['pressure'] = np.array(value)

    @property
    def temperature(self):
        return self._data.get('temperature')

    @temperature.setter
    def temperature(self, value):
        if value is not None:
            self._data['temperature'] = np.array(value)

    def to_json(self):
        return {
            'name': self.recording_name,
            'version': self.version,
            'recorded': self.start_time.strftime('%Y-%m-%d %H:%M:%S'),
            'client_settings': self.client_settings,
            'calibration_parameters': self.calibration_parameters,
            'data': {
                'timestamps': self.timestamps.tolist() if self.timestamps is not None else None,
                'accelerometer': self.accelerometer.tolist() if self.accelerometer is not None else None,
                'gyroscope': self.gyroscope.tolist() if self.gyroscope is not None else None,
                'magnetometer': self.magnetometer.tolist() if self.magnetometer is not None else None,
                'pressure': self.pressure.tolist() if self.pressure is not None else None,
                'temperature': self.temperature.tolist() if self.temperature is not None else None,
            }
        }

    @classmethod
    def from_json(cls, doc):
        out = cls(datetime.datetime.strptime(doc.get('recorded'), '%Y-%m-%d %H:%M:%S'),
            doc.get('client_settings'), doc.get('calibration_parameters'))
        out.recording_name = doc.get('name')
        out.version = doc.get('version', version)
        if out.version.get('pyberryimu') is None:
            out.version['pyberryimu'] = version

        out.timestamps = doc.get('data', {}).get('timestamps')
        out.accelerometer = doc.get('data', {}).get('accelerometer')
        out.gyroscope = doc.get('data', {}).get('gyroscope')
        out.magnetometer = doc.get('data', {}).get('magnetometer')
        out.pressure = doc.get('data', {}).get('pressure')
        out.temperature = doc.get('data', {}).get('temperature')

        return out

    def save(self, file_path):
        with open(os.path.abspath(file_path), 'wt') as f:
            json.dump(self.to_json(), f, indent=2)

    @classmethod
    def load(cls, file_path):
        with open(os.path.abspath(file_path), 'rt') as f:
            doc = json.load(f)
        return cls.from_json(doc)
