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

from pyberryimu.client import BerryIMUClient


def continuous_sampling_test(client):
    for k in xrange(10):
        print("Acc: {0}, Gyro: {1}".format(client.read_accelerometer(), client.read_gyroscope()))


def main():
    with BerryIMUClient() as client:
        continuous_sampling_test(client)
