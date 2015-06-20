#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
:mod:`collector`
==================

.. module:: collector
   :platform: Unix, Windows
   :synopsis: 

.. moduleauthor:: hbldh <henrik.blidh@nedomkull.com>

Created on 2015-06-21, 00:34

"""

from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import

import os
from operator import add

import numpy as np

from pyberryimu.client import BerryIMUClient
from pyberryimu.collector import TimedDataCollectionTool
from pyberryimu.container import DataContainer


def main():
    with BerryIMUClient() as client:

        def read_tri_sensor_array():
            return client.read_accelerometer(), client.read_gyroscope(), client.read_magnetometer()

        tdct = TimedDataCollectionTool(read_tri_sensor_array, 100, 10)
        out = tdct.run()
        dc = DataContainer(start_time=out[0],
                           client_settings=client.get_settings(),
                           calibration_parameters=None)
    dc.timestamps = out[1]
    data = np.array([add(*map(list, x)) for x in out[2]])
    dc.accelerometer = data[:3, :]
    dc.gyroscope = data[3:6, :]
    dc.magnetometer = data[6:, :]
    dc.save(os.path.expanduser('~/pyberryimu_test_data'))

if __name__ == "__main__":
    main()
