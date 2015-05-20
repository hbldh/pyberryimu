#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
:mod:`calibration`
==================

.. module:: calibration
   :platform: Unix, Windows
   :synopsis: 

.. moduleauthor:: hbldh <henrik.blidh@nedomkull.com>

Created on 2015-05-19, 22:52

"""

from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import

import os
import time
import math


def calibrate_accelerometer(client, save_to_file=True):
    """Perform rudimentary sensitivity and zero-G offset calibration

    :param client: The BerryIMU client to calibrate with.
    :type client: :py:class:`pyberryimu.client.BerryIMUClient`
    :return: The calibration parameters.
    :rtype: dict

    """
    output = {'x': {'values': [], 'sensitivity': None, 'scale': None, 'zero': None},
              'y': {'values': [], 'sensitivity': None, 'scale': None, 'zero': None},
              'z': {'values': [], 'sensitivity': None, 'scale': None, 'zero': None}}

    def _wait_for_compliance():
        keep_waiting = 10
        while keep_waiting > 0:
            a = client.read_accelerometer()
            norm_a = _norm(a)
            norm_diff = math.fabs(math.fabs(a[index]) - norm_a) / norm_a

            if norm_diff < 0.05 and cmp(a[index], 0) == side:
                keep_waiting -= 1
            else:
                keep_waiting = 10
            time.sleep(0.1)


    axes_names = ['x', 'y', 'z']
    for index in xrange(3):
        for side in [-1, 1]:
            print('Position BerryIMU {0} axis {1}...'.format(
                axes_names[index], 'downwards' if side < 0 else 'upwards'))
            _wait_for_compliance()

            print('Staring calibration of BerryIMU {0} axis {1} ({2})...'.format(
                axes_names[index], 'downwards' if side < 0 else 'upwards', client.read_accelerometer()))
            acc_values = []
            t = time.time()
            while (time.time() - t) < 3:
                acc_values.append(client.read_accelerometer()[index])
            output[axes_names[index]]['values'].append(sum(acc_values) / len(acc_values))

        output[axes_names[index]]['sensitivity'] = sum(output[axes_names[index]]['values']) / 2
        output[axes_names[index]]['scale'] = 2 / sum(map(math.fabs, output[axes_names[index]]['values']))
        output[axes_names[index]]['zero'] = sum(output[axes_names[index]]['values'])

    if save_to_file:
        try:
            print("Writing accelerometer calibration data to file...")
            import json
            with open(os.path.expanduser('~/.pyberryimu_acc'), 'w') as f:
                json.dump(output, f, indent=2)
        except:
            print("Failed to write to file!")

    print("Accelerometer calibration done.")

    return output


def _norm(a):
    return math.sqrt(sum(map(lambda x: x ** 2, a)))
