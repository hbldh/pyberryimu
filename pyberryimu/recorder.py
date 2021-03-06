#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
:mod:`collector`
==================

.. module:: collector
   :platform: Unix, Windows
   :synopsis: 

.. moduleauthor:: hbldh <henrik.blidh@nedomkull.com>

Created on 2015-05-30, 22:54

"""

from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import

import time
import datetime
import itertools

import numpy as np

from pyberryimu.container import BerryIMUDataContainer


class BerryIMURecorder(object):
    """Class for continuously recording IMU data from the BerryIMU."""

    def __init__(self, client, frequency, duration):
        """Constructor for BerryIMURecorder
        
        :param client: The PyBerryIMU client to record with.
        :type client: :py:class:`pyberryimu.client.BerryIMUClient`
        :param frequency: The frequency / data rate to record data at.
        :type frequency: int
        :param duration: Number of seconds the recoding should last.
        :type duration: int
        
        """
        self.client = client
        self.frequency = frequency
        self.duration = duration

    def _record(self, callback_function):
        timestamps = []
        data = []
        period = 1 / self.frequency

        def g_tick():
            t = time.time()
            count = 0
            while True:
                count += 1
                yield max(t + count * period - time.time(), 0)
        g = g_tick()

        start_dt = datetime.datetime.now()
        start_t = time.time()
        while True:
            time.sleep(g.next())
            t = time.time()
            timestamps.append(t)
            data.append(callback_function())
            if (t - start_t) > self.duration:
                break
        return start_dt, timestamps, data

    def record(self, acc=True, gyro=True, mag=True, pres=False, temp=False):
        """Main recording method.

        :param acc: Record accelerometer values.
        :type acc: bool
        :param gyro: Record gyroscope values.
        :type gyro: bool
        :param mag: Record magnetometer values.
        :type mag: bool
        :param pres: Record pressure values.
        :type pres: bool
        :param temp: Record temperature values.
        :type temp: bool
        :return: The recorded data container.
        :rtype: :py:class:`pyberryimu.container.BerryIMUDataContainer`

        """

        sensor_methods_to_call = []
        if acc:
            sensor_methods_to_call.append(self.client.read_accelerometer)
        if gyro:
            sensor_methods_to_call.append(self.client.read_gyroscope)
        if mag:
            sensor_methods_to_call.append(self.client.read_magnetometer)
        if pres:
            # Define a simple wrapping function to output array instead of scalar.
            def pressure_reader():
                return [self.client.read_pressure(), ]
            sensor_methods_to_call.append(pressure_reader)
        if temp:
            # Define a simple wrapping function to output array instead of scalar.
            def temperature_reader():
                return [self.client.read_temperature(), ]
            sensor_methods_to_call.append(temperature_reader)

        def recording_function():
            """Read all sensors,concatenate all data lists to one list and return it.

            :return: The data read.
            :rtype: list

            """
            return list(itertools.chain(*[f() for f in sensor_methods_to_call]))
        
        def finalizing_function(container, data):
            """A method for parsing recorded data to proper container positions.

            :param container: The container to store the recorded data in.
            :type container: :py:class:`pyberryimu.container.BerryIMUDataContainer`
            :param data: The recorded data.
            :type data: array or tuple
            :return: The data container.
            :rtype: :py:class:`pyberryimu.container.BerryIMUDataContainer`

            """
            container.timestamps = data[1]
            data = np.array(data[2])
            n = 0
            if acc:
                container.accelerometer = data[:, n:n+3]
                n += 3
            if gyro:
                container.gyroscope = data[:, n:n + 3]
                n += 3
            if mag:
                container.magnetometer = data[:, n:n + 3]
                n += 3
            if pres:
                container.pressure = data[:, n]
                n += 1
            if temp:
                container.temperature = data[:, n]
                n += 3

            # Simple check for deviant recording frequency.
            mean_recording_freq = np.mean(1/np.diff(container.timestamps))
            if np.abs((mean_recording_freq - self.frequency) / self.frequency) > 0.05:
                print("Recording deviation detected: Desired freq "
                      "was {0} Hz, achieved was {1:.2f} Hz.".format(self.frequency, mean_recording_freq))

            return data_obj

        out = self._record(recording_function)
        data_obj = BerryIMUDataContainer(out[0], self.client.get_settings(), self.client.calibration_object.to_json())
        return finalizing_function(data_obj, out)

    def record_generic_callback(self, callback_function, finalizing_function):
        """Recording with generic functions.

        :param callback_function:
        :type callback_function: :py:class:`function`
        :param finalizing_function: A method for restructuring the obtained
            data into a :py:class:`pyberryimu.container.BerryIMUDataContainer` and returning it.
        :type finalizing_function: :py:class:`function`
        :return: The recorded data object.
        :rtype: :py:class:`pyberryimu.container.BerryIMUDataContainer`

        """
        out = self._record(callback_function)
        data_obj = BerryIMUDataContainer(out[0], self.client.get_settings(), self.client.calibration_object.to_json())
        return finalizing_function(data_obj, out)
