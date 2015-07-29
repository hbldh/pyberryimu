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

import time

from smbus import SMBus

from pyberryimu.exc import PyBerryIMUError
from pyberryimu.sensors import LSM9DS0, BMP180
from pyberryimu.calibration.base import BerryIMUCalibration


class BerryIMUClient(object):
    """Client for using the BerryIMU from Python.

    Settings can be applied by sending in a dict at initialisation.
    The following keys and corresponding valid values govern the settings, where
    values in <> denotes default values.

    Accelerometer:

    * 'data_rate' [0, 3.125, 6.25, 12.5, 25, 50, 100, <200>, 400, 800, 1600]
    * 'continuous_update': [True, <False>],
    * 'enabled_x': [<True>, False],
    * 'enabled_y': [<True>, False],
    * 'enabled_z': [<True>, False],
    * 'anti_alias': [<773>, 194, 362, 50],
    * 'full_scale': [2, 4, 6, <8>, 16],
    * 'self_test': [<0>, 1, -1, 'X']

    Gyroscope:

    * 'data_rate' [95, <190>, 380, 760]
    * 'bandwidth_level': [<0>, 1, 2, 3],
    * 'powerdown_mode': [True, <False>],
    * 'enabled_x': [<True>, False],
    * 'enabled_y': [<True>, False],
    * 'enabled_z': [<True>, False],
    * 'continuous_update': [True, <False>],
    * 'little_endian': [True, <False>],
    * 'full_scale': [245, <500>, 2000],
    * 'self_test': [<0>, 1, -1]

    Magnetometer:

    * 'data_rate' [3.125, 6.25, 12.5, 25, <50>, 100]
    * 'full_scale': [2, 4, 8, <12>],
    * 'sensor_mode': [<0>, 1, -1]

    Read more about these settings in the
    `LSM9DS0 data sheet <http://ozzmaker.com/wp-content/uploads/2014/12/LSM9DS0.pdf>`_.

    For the barometric pressure sensor BMP180, docstring will be written.
    `BMP180 data sheet <http://ozzmaker.com/wp-content/uploads/2015/01/BMP180-DS000-09.pdf>`_

    """

    def __init__(self, bus=1, settings=None):
        """Constructor for BerryIMUClient"""

        self._bus = None
        self._calibration_object = BerryIMUCalibration()

        # Init time settings.
        self._bus_no = bus
        self._acc_setup = self._create_accelerometer_settings_dict(
            settings.get('accelerometer', {}) if settings is not None else {})
        self._gyro_setup = self._create_gyroscope_settings_dict(
            settings.get('gyroscope', {}) if settings is not None else {})
        self._mag_setup = self._create_magnetometer_settings_dict(
            settings.get('magnetometer', {}) if settings is not None else {})

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

    @property
    def calibration_object(self):
        return self._calibration_object

    @calibration_object.setter
    def calibration_object(self, new_calibration):
        # TODO: Check if calibration and current client share vital settings such as full scales.
        self._calibration_object = new_calibration

    def open(self):
        try:
            self._bus = SMBus(self._bus_no)
        except IOError as e:
            if str(e) == '2':
                raise PyBerryIMUError("/dev/i2c-{0} not found. (IOError 2)".format(self._bus_no))
            elif str(e) == '5':
                raise PyBerryIMUError("I2C Input/Output error. (IOError 5)".format(self._bus_no))
            elif str(e) == '13':
                raise PyBerryIMUError("Permission to read and/or write to "
                                      "/dev/i2c-{0} missing. (IOError 13)".format(self._bus_no))
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

    def _create_accelerometer_settings_dict(self, setup_dict):
        return {
            'data_rate': setup_dict.get('data_rate', 200),
            'continuous_update': setup_dict.get('continuous_update', False),
            'enabled_z': setup_dict.get('enabled_z', True),
            'enabled_y': setup_dict.get('enabled_y', True),
            'enabled_x': setup_dict.get('enabled_x', True),
            'anti_alias': setup_dict.get('anti_alias', 773),
            'full_scale': setup_dict.get('full_scale', 8),
            'self_test': setup_dict.get('self_test', 0)
        }

    def _init_accelerometer(self):
        """Initialize the accelerometer according to the settings document sent in."""
        # TODO: Better init handling!

        reg1_value = (
            LSM9DS0.get_accelerometer_data_rate_bits(self._acc_setup.get('data_rate')) +
            ('0' if self._acc_setup.get('continuous_update') else '1') +
            ('1' if self._acc_setup.get('enabled_z') else '0') +
            ('1' if self._acc_setup.get('enabled_y') else '0') +
            ('1' if self._acc_setup.get('enabled_x') else '0')
        )
        # Write data rate, enabled axes and block data update.
        self._write(LSM9DS0.ACC_ADDRESS, LSM9DS0.CTRL_REG1_XM, int(reg1_value, 2))

        reg2_value = (
            LSM9DS0.get_accelerometer_anti_alias_filter_bits(self._acc_setup.get('anti_alias')) +
            LSM9DS0.get_accelerometer_full_scale_bits(self._acc_setup.get('full_scale')) +
            LSM9DS0.get_accelerometer_self_test_bits(self._acc_setup.get('self_test')) +
            '0'  # SPI Serial Interface Mode selection
        )
        # Write anti-alias filter bandwidth, acceleration full scale and self-test mode.
        self._write(LSM9DS0.ACC_ADDRESS, LSM9DS0.CTRL_REG2_XM, int(reg2_value, 2))

    def _create_gyroscope_settings_dict(self, setup_dict):
        return {
            'data_rate': setup_dict.get('data_rate', 190),
            'bandwidth_level': setup_dict.get('bandwidth_level', 0),
            'powerdown_mode': setup_dict.get('powerdown_mode', False),
            'enabled_z': setup_dict.get('enabled_z', True),
            'enabled_y': setup_dict.get('enabled_y', True),
            'enabled_x': setup_dict.get('enabled_x', True),
            'continuous_update': setup_dict.get('continuous_update', False),
            'little_endian': setup_dict.get('little_endian', False),
            'full_scale': setup_dict.get('full_scale', 500),
            'self_test': setup_dict.get('self_test', 0)
        }

    def _init_gyroscope(self):
        """Initialize the gyroscope according to the settings document sent in."""
        # TODO: Better init handling!

        reg1_value = (
            LSM9DS0.get_gyroscope_data_rate_bits(self._gyro_setup.get('data_rate')) +
            LSM9DS0.get_gyroscope_bandwidth_bits(self._gyro_setup.get('bandwidth_level')) +
            ('0' if self._gyro_setup.get('powerdown_mode') else '1') +
            ('1' if self._gyro_setup.get('enabled_z') else '0') +
            ('1' if self._gyro_setup.get('enabled_y') else '0') +
            ('1' if self._gyro_setup.get('enabled_x') else '0')
        )
        self._write(LSM9DS0.GYR_ADDRESS, LSM9DS0.CTRL_REG1_G, int(reg1_value, 2))

        # TODO: Add setup for high-pass filter, LSM9DS0.CTRL_REG2_G and LSM9DS0.CTRL_REG5_G

        reg4_value = (
            ('0' if self._gyro_setup.get('continuous_update') else '1') +
            ('1' if self._gyro_setup.get('little_endian') else '0') +
            LSM9DS0.get_gyroscope_full_scale_bits(self._gyro_setup.get('full_scale')) +
            '0' +  # Unused bit.
            LSM9DS0.get_gyroscope_self_test_bits(self._gyro_setup.get('self_test')) +
            '0'  # SPI Serial Interface Mode selection
        )
        self._write(LSM9DS0.GYR_ADDRESS, LSM9DS0.CTRL_REG4_G, int(reg4_value, 2))

    def _create_magnetometer_settings_dict(self, setup_dict):
        return {
            'enabled_temp': setup_dict.get('enabled_temp', True),
            'data_rate': setup_dict.get('data_rate', 50),
            'full_scale': setup_dict.get('full_scale', 12),
            'sensor_mode': setup_dict.get('sensor_mode', 0),
        }

    def _init_magnetometer(self):
        """Initialize the magnetometer according to the settings document sent in."""
        reg5_value = (
            ('1' if self._gyro_setup.get('enabled_temp') else '0') +
            '11' +  # Magnetic resolution selection (hardcoded to high resolution!)
            LSM9DS0.get_magnetometer_data_rate_bits(self._mag_setup.get('data_rate')) +
            '00'  # Latch interrupts disabled right.
        )
        self._write(LSM9DS0.MAG_ADDRESS, LSM9DS0.CTRL_REG5_XM, int(reg5_value, 2))

        reg6_value = (
            '0' +  # Unused bits
            LSM9DS0.get_magnetometer_full_scale_bits(self._mag_setup.get('full_scale')) +
            '00000'  # Unused bits
        )
        self._write(LSM9DS0.MAG_ADDRESS, LSM9DS0.CTRL_REG6_XM, int(reg6_value, 2))

        reg7_value = (
            '00' +  # Accelerometer high-pass filter disabled.
            '0' +  # Filtered acceleration data selection bypassed.
            '00' +  # Unused bits
            '0' +  # Magnetic data low-power mode disabled.
            LSM9DS0.get_magnetometer_sensor_mode_bits(self._mag_setup.get('sensor_mode'))
        )
        self._write(LSM9DS0.MAG_ADDRESS, LSM9DS0.CTRL_REG7_XM, int(reg7_value, 2))

    def _init_barometric_pressure_sensor(self):
        """Initialize the Barometric Pressure Sensor."""
        self._set_bmp180_calibration_values()
        # TODO: Better init handling!
        self.__OVERSAMPLING = 3  # 0..3

    def get_settings(self):
        settings = {
            'accelerometer': {
                'reg1': "{0:08b}".format(self.bus.read_byte_data(LSM9DS0.ACC_ADDRESS, LSM9DS0.CTRL_REG1_XM)),
                'reg2': "{0:08b}".format(self.bus.read_byte_data(LSM9DS0.ACC_ADDRESS, LSM9DS0.CTRL_REG2_XM)),
            },
            'gyroscope': {
                'reg1': "{0:08b}".format(self.bus.read_byte_data(LSM9DS0.GYR_ADDRESS, LSM9DS0.CTRL_REG1_G)),
                'reg4': "{0:08b}".format(self.bus.read_byte_data(LSM9DS0.GYR_ADDRESS, LSM9DS0.CTRL_REG4_G)),
            },
            'magnetometer': {
                'reg5': "{0:08b}".format(self.bus.read_byte_data(LSM9DS0.MAG_ADDRESS, LSM9DS0.CTRL_REG5_XM)),
                'reg6': "{0:08b}".format(self.bus.read_byte_data(LSM9DS0.MAG_ADDRESS, LSM9DS0.CTRL_REG6_XM)),
                'reg7': "{0:08b}".format(self.bus.read_byte_data(LSM9DS0.MAG_ADDRESS, LSM9DS0.CTRL_REG7_XM)),
            }
        }
        settings['accelerometer'].update(self._acc_setup)
        settings['gyroscope'].update(self._gyro_setup)
        settings['magnetometer'].update(self._mag_setup)
        return settings

    # BMP180 specific methods.

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
        """Timestamp right now Epoch time.

        :return: Epoch timestamp
        :rtype: float

        """
        # TODO: Make timezone independent...
        return time.time()

    def read_accelerometer(self):
        """Method for reading values from the accelerometer.

        :return: The X, Y, and Z values of the accelerometer.
        :rtype: tuple

        """
        return self.calibration_object.transform_accelerometer_values(
            (self._read(LSM9DS0.ACC_ADDRESS, LSM9DS0.OUT_X_L_A, LSM9DS0.OUT_X_H_A),
             self._read(LSM9DS0.ACC_ADDRESS, LSM9DS0.OUT_Y_L_A, LSM9DS0.OUT_Y_H_A),
             self._read(LSM9DS0.ACC_ADDRESS, LSM9DS0.OUT_Z_L_A, LSM9DS0.OUT_Z_H_A)))

    def read_gyroscope(self):
        """Method for reading values from the gyroscope.

        :return: The X, Y, and Z values of the gyroscope.
        :rtype: tuple

        """
        return self.calibration_object.transform_gyroscope_values(
            (self._read(LSM9DS0.GYR_ADDRESS, LSM9DS0.OUT_X_L_G, LSM9DS0.OUT_X_H_G),
             self._read(LSM9DS0.GYR_ADDRESS, LSM9DS0.OUT_Y_L_G, LSM9DS0.OUT_Y_H_G),
             self._read(LSM9DS0.GYR_ADDRESS, LSM9DS0.OUT_Z_L_G, LSM9DS0.OUT_Z_H_G)))

    def read_magnetometer(self):
        """Method for reading values from the magnetometer.

        :return: The X, Y, and Z values of the magnetometer.
        :rtype: tuple

        """
        return self.calibration_object.transform_magnetometer_values(
            (self._read(LSM9DS0.MAG_ADDRESS, LSM9DS0.OUT_X_L_M, LSM9DS0.OUT_X_H_M),
             self._read(LSM9DS0.MAG_ADDRESS, LSM9DS0.OUT_Y_L_M, LSM9DS0.OUT_Y_H_M),
             self._read(LSM9DS0.MAG_ADDRESS, LSM9DS0.OUT_Z_L_M, LSM9DS0.OUT_Z_H_M)))

    def read_temperature_LSM9DS0(self):
        """Method for reading temperature values from the LSM9DS0 chip.

        Temperature value is stored as a 12-bit, right justified value, hence the
        bit mask on the high bit.

        :return: Temperature value.
        :rtype: int

        """
        value = (self.bus.read_byte_data(LSM9DS0.MAG_ADDRESS, LSM9DS0.OUT_TEMP_L_XM) | (
            (self.bus.read_byte_data(LSM9DS0.MAG_ADDRESS, LSM9DS0.OUT_TEMP_H_XM) & 0b00001111) << 8))
        value = value if value < 2048 else value - 4096

        # Convert to degrees Celsius according to data sheet specs: 8 LSB/deg C
        return value / 8

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

