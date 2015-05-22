#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
:mod:`client`
==================

.. module:: client
   :platform: Unix, Windows
   :synopsis: 

.. moduleauthor:: hbldh <henrik.blidh@nedomkull.com>

Created on 2015-05-18, 11:51

"""

from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import

import math
import time
import datetime

from smbus import SMBus

from pyberryimu.exc import PyBerryIMUError
from pyberryimu.sensors import LSM9DS0, BMP180


class BerryIMUClient(object):
    """Client for using the BerryIMU from Python.

    Datasheets for BerryIMU::

        * `LSM9DS0 reference sheet
            <http://ozzmaker.com/wp-content/uploads/2014/12/LSM9DS0.pdf>`_
        * `BMP180 reference sheet
            <http://ozzmaker.com/wp-content/uploads/2015/01/BMP180-DS000-09.pdf>`_

    """

    def __init__(self, bus=1, acc_setup=1, gyro_setup=1, mag_setup=1, raw_output=False):
        """Constructor for BerryIMUClient"""

        self._bus = None

        # Init time settings.
        self._bus_no = bus
        self._acc_setup = acc_setup
        self._gyro_setup = gyro_setup
        self._mag_setup = mag_setup
        self._raw_output = raw_output

        # BMP180 calibration values.
        self.__ac1 = None
        self.__ac2 = None
        self.__ac3 = None
        self.__ac4 = None
        self.__ac5 = None
        self.__ac6 = None
        self.__b1 = None
        self.__b2 = None
        self.__mb = None
        self.__mc = None
        self.__md = None

    @property
    def bus(self):
        if self._bus is not None:
            return self._bus
        else:
            self.open()
            return self.bus

    def open(self):
        try:
            self._bus = SMBus(self._bus_no)
        except IOError as e:
            if str(e) == '2':
                raise PyBerryIMUError("/dev/i2c-{0} not found.".format(self._bus_no))
            elif str(e) == '13':
                raise PyBerryIMUError("Permission to read and/or write to /dev/i2c-{0} missing.".format(self._bus_no))
            else:
                raise PyBerryIMUError('Unhandled IOError: {0}'.format(e))
        except Exception as e:
            raise PyBerryIMUError('Unhandled {0}: {1}'.format(type(e), e))
        else:
            self._init_accelerometer()
            self._init_gyroscope()
            self._init_magnetometer()
            self._init_barometric_pressure_sensor()

    def close(self):
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception as e:
                # TODO: Test what errors can occur and handle these better.
                print("Exception at closing of i2c bus: {0}".format(e))

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    # Initialisation methods

    def _init_accelerometer(self):
        """Initialize the accelerometer according to the settings flag stored."""
        # TODO: Better init handling!
        if self._acc_setup == 1:
            # z,y,x axis enabled, continuous update, 100Hz data rate
            self._write(LSM9DS0.ACC_ADDRESS, LSM9DS0.CTRL_REG1_XM, 0b01100111)
            # +/- 16G full scale
            self._write(LSM9DS0.ACC_ADDRESS, LSM9DS0.CTRL_REG2_XM, 0b00100000)
        else:
            raise PyBerryIMUError("Invalid Accelerometer setup flag: {0}.".format(self._acc_setup))

    def _init_gyroscope(self):
        """Initialize the gyroscope according to the settings flag stored."""
        # TODO: Better init handling!
        if self._gyro_setup == 1:
            # Normal power mode, all axes enabled
            self._write(LSM9DS0.GYR_ADDRESS, LSM9DS0.CTRL_REG1_G, 0b00001111)
            # Continuous update, 2000 dps full scale
            self._write(LSM9DS0.GYR_ADDRESS, LSM9DS0.CTRL_REG4_G, 0b00110000)

            # Conversion constants for this setting [deg/s/LSB]
            # TODO: Wrap G_GAIN in automatic calculation of value.
            self.__G_GAIN = 0.070
            self.__RAD_TO_DEG = math.degrees(1)
            self.__PI = math.pi
            # TODO: Study Loop period dependency.
            # Loop period = 41ms.   This needs to match the time it takes each loop to run
            self.__LP = 0.041
        else:
            raise PyBerryIMUError("Invalid Gyroscope setup flag: {0}.".format(self._gyro_setup))

    def _init_magnetometer(self):
        """Initialize the magnetometer according to the settings flag stored."""
        # TODO: Better init handling!
        if self._mag_setup == 1:
            # Temp enable, M data rate = 50Hz
            self._write(LSM9DS0.MAG_ADDRESS, LSM9DS0.CTRL_REG5_XM, 0b11110000)
            # +/-12gauss
            self._write(LSM9DS0.MAG_ADDRESS, LSM9DS0.CTRL_REG6_XM, 0b01100000)
            # Continuous-conversion mode
            self._write(LSM9DS0.MAG_ADDRESS, LSM9DS0.CTRL_REG7_XM, 0b00000000)
        else:
            raise PyBerryIMUError("Invalid Magnetometer setup flag: {0}.".format(self._mag_setup))

    def _init_barometric_pressure_sensor(self):
        """Initialize the Barometric Pressure Sensor."""
        self._set_bmp180_calibration_values()
        # TODO: Better init handling!
        self.__OVERSAMPLING = 3  # 0..3

    def get_bmp180_chip_id_and_version(self):
        """Gets Chip ID and version for the BMP180 sensor.

        :return: Chip ID and Version number
        :rtype: tuple

        """
        return self.bus.read_i2c_block_data(BMP180.ADDRESS, BMP180.CHIP_ID_REG, 2)

    def _set_bmp180_calibration_values(self):
        """Read, parse and store calibration EEPROM data to attributes."""

        bmp180_calibration = self.bus.read_i2c_block_data(BMP180.ADDRESS, BMP180.CALIB_DATA_REG, 22)
        vals = [msb + lsb for msb, lsb in zip(map(lambda x: x << 8, bmp180_calibration[::2]),
                                              bmp180_calibration[1::2])]
        for i in [0, 1, 2, 6, 7, 8, 9, 10]:
            if vals[i] > 2 ** 15 - 1:
                vals[i] -= 2 ** 16

        self.__ac1 = vals[0]
        self.__ac2 = vals[1]
        self.__ac3 = vals[2]
        self.__ac4 = vals[3]
        self.__ac5 = vals[4]
        self.__ac6 = vals[5]
        self.__b1 = vals[6]
        self.__b2 = vals[7]
        self.__mb = vals[8]
        self.__mc = vals[9]
        self.__md = vals[10]

    # Methods for writing to BerryIMU.

    def _write(self, address, register, value):
        self.bus.write_byte_data(address, register, value)
        return -1

    # Methods for reading from BerryIMU.

    def _read(self, address, register_low_bit, register_high_bit):
        value = (self.bus.read_byte_data(address, register_low_bit) |
                 (self.bus.read_byte_data(address, register_high_bit) << 8))

        return value if value < 32768 else value - 65536

    @property
    def timestamp(self):
        """Timestamp right now in UTC Epoch time.

        :return: UTC Epoch timestamp
        :rtype: float

        """
        return time.mktime(datetime.datetime.utcnow().timetuple())

    def read_accelerometer(self):
        """Method for reading values from the accelerometer.

        :return: The X, Y, and Z values of the accelerometer.
        :rtype: tuple

        """
        return (self._read(LSM9DS0.ACC_ADDRESS, LSM9DS0.OUT_X_L_A, LSM9DS0.OUT_X_H_A),
                self._read(LSM9DS0.ACC_ADDRESS, LSM9DS0.OUT_Y_L_A, LSM9DS0.OUT_Y_H_A),
                self._read(LSM9DS0.ACC_ADDRESS, LSM9DS0.OUT_Z_L_A, LSM9DS0.OUT_Z_H_A))

    def read_gyroscope(self):
        """Method for reading values from the gyroscope.

        :return: The X, Y, and Z values of the gyroscope.
        :rtype: tuple

        """
        return (self._read(LSM9DS0.GYR_ADDRESS, LSM9DS0.OUT_X_L_G, LSM9DS0.OUT_X_H_G),
                self._read(LSM9DS0.GYR_ADDRESS, LSM9DS0.OUT_Y_L_G, LSM9DS0.OUT_Y_H_G),
                self._read(LSM9DS0.GYR_ADDRESS, LSM9DS0.OUT_Z_L_G, LSM9DS0.OUT_Z_H_G))

    def read_magnetometer(self):
        """Method for reading values from the magnetometer.

        :return: The X, Y, and Z values of the magnetometer.
        :rtype: tuple

        """
        return (self._read(LSM9DS0.MAG_ADDRESS, LSM9DS0.OUT_X_L_M, LSM9DS0.OUT_X_H_M),
                self._read(LSM9DS0.MAG_ADDRESS, LSM9DS0.OUT_Y_L_M, LSM9DS0.OUT_Y_H_M),
                self._read(LSM9DS0.MAG_ADDRESS, LSM9DS0.OUT_Z_L_M, LSM9DS0.OUT_Z_H_M))

    def read_temperature(self):
        """Method for reading temperature values from the barometric pressure sensor.

        Reference for reading to temperature calculations:
        `BMP180 reference sheet
        <http://ozzmaker.com/wp-content/uploads/2015/01/BMP180-DS000-09.pdf>`_

        :return: The temperature value.
        :rtype: int

        """
        self._write(BMP180.ADDRESS, BMP180.WRITE_REG, 0x2E)
        time.sleep(0.005)
        msb, lsb = self.bus.read_i2c_block_data(BMP180.ADDRESS, BMP180.READ_REG, 2)
        ut = (msb << 8) + lsb

        x1 = ((ut - self.__ac6) * self.__ac5) >> 15
        x2 = (self.__mc << 11) // (x1 + self.__md)
        b5 = x1 + x2
        t = (b5 + 8) >> 4
        return t / 10.0

    def read_pressure(self):
        """Method for reading pressure value from the barometric pressure sensor.

        Reference for reading to pressure calculations:
        `BMP180 reference sheet
        <http://ozzmaker.com/wp-content/uploads/2015/01/BMP180-DS000-09.pdf>`_


        :return: The pressure value.
        :rtype: int

        """
        self._write(BMP180.ADDRESS, BMP180.WRITE_REG, 0x2E)
        time.sleep(0.005)
        msb, lsb = self.bus.read_i2c_block_data(BMP180.ADDRESS, BMP180.READ_REG, 2)
        ut = (msb << 8) + lsb

        self._write(BMP180.ADDRESS, BMP180.WRITE_REG, 0x34 + (self.__OVERSAMPLING << 6))
        time.sleep(0.04)
        msb, lsb, xsb = self.bus.read_i2c_block_data(BMP180.ADDRESS, BMP180.READ_REG, 3)
        up = ((msb << 16) + (lsb << 8) + xsb) >> (8 - self.__OVERSAMPLING)

        x1 = ((ut - self.__ac6) * self.__ac5) >> 15
        x2 = (self.__mc << 11) // (x1 + self.__md)
        b5 = x1 + x2

        b6 = b5 - 4000
        b62 = b6 * b6 >> 12
        x1 = (self.__b2 * b62) >> 11
        x2 = self.__ac2 * b6 >> 11
        x3 = x1 + x2
        b3 = (((self.__ac1 * 4 + x3) << self.__OVERSAMPLING) + 2) >> 2

        x1 = self.__ac3 * b6 >> 13
        x2 = (self.__b1 * b62) >> 16
        x3 = ((x1 + x2) + 2) >> 2
        b4 = (self.__ac4 * (x3 + 32768)) >> 15
        b7 = (up - b3) * (50000 >> self.__OVERSAMPLING)

        p = (b7 * 2) // b4

        x1 = (p >> 8) * (p >> 8)
        x1 = (x1 * 3038) >> 16
        x2 = (-7357 * p) >> 16
        p += (x1 + x2 + 3791) >> 4

        return p / 100.0
