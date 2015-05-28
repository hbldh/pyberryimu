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


def max_freq_test(client, n=2000):
    t = time.time()
    acc_data = []
    for k in xrange(n):
        acc_data.append(client.read_accelerometer())
    elapsed = time.time() - t
    print("Max accelerometer read frequency: {0:.2f} Hz".format(2000 / elapsed))

    t = time.time()
    gyro_data = []
    for k in xrange(n):
        gyro_data.append(client.read_gyroscope())
    elapsed = time.time() - t
    print("Max gyroscope read frequency: {0:.2f} Hz".format(2000 / elapsed))

    t = time.time()
    mag_data = []
    for k in xrange(n):
        mag_data.append(client.read_magnetometer())
    elapsed = time.time() - t
    print("Max magnetometer read frequency: {0:.2f} Hz".format(2000 / elapsed))

    t = time.time()
    data = []
    for k in xrange(n):
        data.append((client.read_accelerometer(), client.read_gyroscope()))
    elapsed = time.time() - t
    print("Max Acc + Gyro read frequency: {0:.2f} Hz".format(2000 / elapsed))

    t = time.time()
    data = []
    for k in xrange(2000):
        data.append((client.read_accelerometer(), client.read_gyroscope(), client.read_magnetometer()))
    elapsed = time.time() - t
    print("Max Acc + Gyro + Mag read frequency: {0:.2f} Hz".format(2000 / elapsed))


def main():
    with BerryIMUClient() as client:
        max_freq_test(client)

if __name__ == "__main__":
    main()
