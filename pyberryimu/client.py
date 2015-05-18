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
from smbus import SMBus
from pyberryimu.exc import PyBerryIMUError
from pyberryimu.sensors import LSM9DS0, BMP180


class BerryIMUClient(object):
    """"""

    def __init__(self, bus=1, acc_setup=1, gyro_setup=1, mag_setup=1, raw_output=False):
        """Constructor for BerryIMUClient"""

        self._bus = None

        # Init time settings.
        self._bus_no = bus
        self._acc_setup = acc_setup
        self._gyro_setup = gyro_setup
        self._mag_setup = mag_setup
        self._raw_output = raw_output

        self._bmp180_calibration = None

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
            # TODO: Handle these undocumented errors better.
            raise
        else:
            self._init_accelerometer()
            self._init_gyroscope()
            self._init_magnetometer()
            return True

    def close(self):
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception as e:
                # TODO: Test what errors can occur and handle these better.
                print("close() exception: {0}".format(e))
        return True

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
            self._write_to_accelerometer(LSM9DS0.CTRL_REG1_XM, 0b01100111)
            # +/- 16G full scale
            self._write_to_accelerometer(LSM9DS0.CTRL_REG2_XM, 0b00100000)
        else:
            raise PyBerryIMUError("Invalid Accelerometer setup flag: {0}.".format(self._acc_setup))

    def _init_gyroscope(self):
        """Initialize the gyroscope according to the settings flag stored."""
        # TODO: Better init handling!
        if self._gyro_setup == 1:
            # Normal power mode, all axes enabled
            self._write_to_gyroscope(LSM9DS0.CTRL_REG1_G, 0b00001111)
            # Continuous update, 2000 dps full scale
            self._write_to_gyroscope(LSM9DS0.CTRL_REG4_G, 0b00110000)

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
            self._write_to_magnetometer(LSM9DS0.CTRL_REG5_XM, 0b11110000)
            # +/-12gauss
            self._write_to_magnetometer(LSM9DS0.CTRL_REG6_XM, 0b01100000)
            # Continuous-conversion mode
            self._write_to_magnetometer(LSM9DS0.CTRL_REG7_XM, 0b00000000)
        else:
            raise PyBerryIMUError("Invalid Magnetometer setup flag: {0}.".format(self._mag_setup))

    def _init_barometric_pressure_sensor(self):
        """Initialize the Barometric Pressure Sensor."""
        # Read whole calibration EEPROM data
        self._bmp180_calibration = self.bus.read_i2c_block_data(BMP180.ADDRESS, BMP180.CALIB_DATA_REG, 22)
        # TODO: Translate calibration bytes to data.

    def get_bmp180_chip_id_and_version(self):
        """Gets Chip ID and version for the BMP180 sensor.

        :return: Chip ID and Version number
        :rtype: tuple

        """
        return self.bus.read_i2c_block_data(BMP180.ADDRESS, BMP180.CHIP_ID_REG, 2)

    # Methods for writing to BerryIMU.

    def _write_to_accelerometer(self, register, value):
        self.bus.write_byte_data(LSM9DS0.ACC_ADDRESS, register, value)
        return -1

    def _write_to_gyroscope(self, register, value):
        self.bus.write_byte_data(LSM9DS0.GYR_ADDRESS, register, value)
        return -1

    def _write_to_magnetometer(self, register, value):
        self.bus.write_byte_data(LSM9DS0.MAG_ADDRESS, register, value)
        return -1

    # Methods for reading from BerryIMU.

    def _read(self, address, register_low_bit, register_high_bit):
        value = (self.bus.read_byte_data(address, register_low_bit) |
                 (self.bus.read_byte_data(address, register_high_bit) << 8))

        return value if value < 32768 else value - 65536

    # TODO: Add conversion to proper units in reading methods.
    # TODO: Add timestamp as utput for all read methods?

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
        # # Convert Gyro raw to degrees per second
        # rate_gyr_x = g_x * G_GAIN
        # rate_gyr_y = g_y * G_GAIN
        # rate_gyr_z = g_z * G_GAIN
        #
        #
        # #Calculate the angles from the gyro. LP = loop period
        # gyroXangle += rate_gyr_x * LP;
        # gyroYangle += rate_gyr_y * LP;
        # gyroZangle += rate_gyr_z * LP;

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
        raise NotImplementedError()

    def read_pressure(self):
        raise NotImplementedError()

