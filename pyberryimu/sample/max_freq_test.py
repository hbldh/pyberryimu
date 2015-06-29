#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
:mod:`test`
==================

.. module:: test
   :platform: Unix, Windows
   :synopsis: 

.. moduleauthor:: hbldh <henrik.blidh@nedomkull.com>

Created on 2015-05-26, 11:22

"""

from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import

import time
from pyberryimu.client import BerryIMUClient


def max_freq_test(client, n=1000):
    t = time.time()
    acc_data = []
    for k in xrange(n):
        acc_data.append(client.read_accelerometer())
    elapsed = time.time() - t
    print("Max Accelerometer read frequency: {0:.2f} Hz".format(n / elapsed))

    t = time.time()
    gyro_data = []
    for k in xrange(n):
        gyro_data.append(client.read_gyroscope())
    elapsed = time.time() - t
    print("Max Gyroscope read frequency: {0:.2f} Hz".format(n / elapsed))

    t = time.time()
    mag_data = []
    for k in xrange(n):
        mag_data.append(client.read_magnetometer())
    elapsed = time.time() - t
    print("Max Magnetometer read frequency: {0:.2f} Hz".format(n / elapsed))

    t = time.time()
    data = []
    for k in xrange(n):
        data.append((client.read_accelerometer(), client.read_gyroscope()))
    elapsed = time.time() - t
    print("Max Acc + Gyro read frequency: {0:.2f} Hz".format(n / elapsed))

    t = time.time()
    data = []
    for k in xrange(n):
        data.append((client.read_accelerometer(), client.read_gyroscope(), client.read_magnetometer()))
    elapsed = time.time() - t
    print("Max Acc + Gyro + Mag read frequency: {0:.2f} Hz".format(n / elapsed))

    t = time.time()
    data = []
    for k in xrange(n // 10):
        data.append(client.read_pressure())
    elapsed = time.time() - t
    print("Max Pressure read frequency: {0:.2f} Hz".format((n // 10) / elapsed))

    t = time.time()
    data = []
    for k in xrange(n // 10):
        data.append((client.read_pressure(), client.read_temperature()))
    elapsed = time.time() - t
    print("Max Pressure + Temperature read frequency: {0:.2f} Hz".format((n // 10) / elapsed))

    t = time.time()
    data = []
    for k in xrange(n // 10):
        data.append((client.read_accelerometer(), client.read_gyroscope(), client.read_magnetometer(),
                     client.read_pressure(), client.read_temperature()))
    elapsed = time.time() - t
    print("Max all-sensor read frequency: {0:.2f} Hz".format((n // 10) / elapsed))


def main():
    with BerryIMUClient() as client:
        max_freq_test(client)

if __name__ == "__main__":
    main()
