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

import numpy as np

from pyberryimu.client import BerryIMUClient
from pyberryimu.calibration.standard import StandardCalibration
from pyberryimu.recorder import BerryIMURecorder
from pyberryimu.container import PyBerryIMUContainer


def main():
    with BerryIMUClient() as client:
        client.calibration_object = StandardCalibration.load()
        brec = BerryIMURecorder(client, frequency=100, duration=3)
        dc = brec.record(acc=True, gyro=True, mag=True, pres=False, temp=False)
    dc.save(os.path.expanduser('~/pyberryimu_test_data'))

if __name__ == "__main__":
    main()
