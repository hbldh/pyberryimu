#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
:mod:`test_client`
==================

.. module:: test_client
   :platform: Unix, Windows
   :synopsis:

.. moduleauthor:: hbldh <henrik.blidh@nedomkull.com>

Created on 2015-07-30

"""

from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import

from mock import Mock, patch

from pyberryimu.sensors import BMP180, LSM9DS0


# Bundles the SMBus mock object from https://github.com/adafruit/Adafruit_Python_GPIO.
class MockSMBus(object):
    # Mock the smbus.SMBus class to record all data written to specific
    # addresses and registers in the _written member.
    def __init__(self):
        # _written will store a dictionary of address to register dictionary.
        # Each register dictionary will store a mapping of register value to
        # an array of all written values (in sequential write order).
        self._written = {}
        self._read = {}

    def _write_register(self, address, register, value):
        self._written.setdefault(address, {}).setdefault(register, []).append(value)

    def _read_register(self, address, register):
        return self._read.get(address).get(register).pop(0)

    def write_byte_data(self, address, register, value):
        self._write_register(address, register, value)

    def write_word_data(self, address, register, value):
        self._write_register(address, register, value >> 8 & 0xFF)
        self._write_register(address, register+1, value & 0xFF)

    def write_i2c_block_data(self, address, register, values):
        for i, value in enumerate(values):
            self._write_register(address, register+i, value & 0xFF)

    def read_byte_data(self, address, register):
        return self._read_register(address, register)

    def read_word_data(self, address, register):
        high = self._read_register(address, register)
        low = self._read_register(address, register+1)
        return (high << 8) | low

    def read_i2c_block_data(self, address, register, length):
        return [self._read_register(address, register + i) for i in range(length)]


def create_device(busnum=1, settings=None):
    # Mock the smbus module and inject it into the global namespace so the
    # pyberryimu module can be imported.  Also inject a mock SMBus
    # instance to be returned by smbus.SMBus function calls.
    smbus = Mock()
    mockbus = MockSMBus()
    smbus.SMBus.return_value = mockbus
    with patch.dict('sys.modules', {'smbus': smbus}):
        from pyberryimu.client import BerryIMUClient
        client = BerryIMUClient(busnum, settings)
        # Write factory calibration data for BMP180 sensor.
        for i, value in enumerate(range(23)):
            mockbus._read.setdefault(BMP180.ADDRESS, {}).setdefault(
                BMP180.CALIB_DATA_REG + i, []).append(value)
        return client, smbus, mockbus


class TestClient(object):

    @staticmethod
    def _test_bits_written(component_name, settings_name, address, register, mask, shift, settings_value, bstr):
        c, smbus, mockbus = create_device(1, {component_name: {settings_name: settings_value}})
        c.open()
        value = (mockbus._written.get(address, {}).get(register, []).pop(0) & mask) >> shift
        assert value == int(bstr, 2)

    def test_correct_init_of_bus(self):
        c, smbus, mockbus = create_device(1, None)
        assert c.bus == mockbus
        smbus.SMBus.assert_called_with(1)

    def test_correct_acc_data_rate_applied(self):
        """Test that Accelerometer data rates are written correctly."""
        for s_val, binstring in LSM9DS0._TABLE_72.items():
            yield (self._test_bits_written, 'accelerometer', 'data_rate',
                   LSM9DS0.ACC_ADDRESS, LSM9DS0.CTRL_REG1_XM, 0b11110000, 4, s_val, binstring)

    def test_correct_acc_cont_update(self):
        """Test that Accelerometer continuous update written correctly."""
        for s_val, binstring in [(True, '0'), (False, '1')]:
            yield (self._test_bits_written, 'accelerometer', 'continuous_update',
                   LSM9DS0.ACC_ADDRESS, LSM9DS0.CTRL_REG1_XM, 0b00001000, 3, s_val, binstring)

    def test_correct_acc_enabled_z(self):
        """Test that Accelerometer settings are written correctly."""
        for s_val, binstring in [(True, '1'), (False, '0')]:
            yield (self._test_bits_written, 'accelerometer', 'enabled_z',
                   LSM9DS0.ACC_ADDRESS, LSM9DS0.CTRL_REG1_XM, 0b00000100, 2, s_val, binstring)

    def test_correct_acc_enabled_y(self):
        """Test that Accelerometer settings are written correctly."""
        for s_val, binstring in [(True, '1'), (False, '0')]:
            yield (self._test_bits_written, 'accelerometer', 'enabled_y',
                   LSM9DS0.ACC_ADDRESS, LSM9DS0.CTRL_REG1_XM, 0b00000010, 1, s_val, binstring)

    def test_correct_acc_enabled_x(self):
        """Test that Accelerometer settings are written correctly."""
        for s_val, binstring in [(True, '1'), (False, '0')]:
            yield (self._test_bits_written, 'accelerometer', 'enabled_x',
                   LSM9DS0.ACC_ADDRESS, LSM9DS0.CTRL_REG1_XM, 0b00000001, 0, s_val, binstring)

    def test_correct_acc_antialias_applied(self):
        """Test that Accelerometer settings are written correctly."""
        for s_val, binstring in LSM9DS0._TABLE_75.items():
            yield (self._test_bits_written, 'accelerometer', 'anti_alias',
                   LSM9DS0.ACC_ADDRESS, LSM9DS0.CTRL_REG2_XM, 0b11000000, 6, s_val, binstring)

    def test_correct_acc_full_scale_applied(self):
        """Test that Accelerometer settings are written correctly."""
        for s_val, binstring in LSM9DS0._TABLE_76.items():
            yield (self._test_bits_written, 'accelerometer', 'full_scale',
                   LSM9DS0.ACC_ADDRESS, LSM9DS0.CTRL_REG2_XM, 0b00111000, 3, s_val, binstring)

    def test_correct_acc_self_test_applied(self):
        """Test that Accelerometer settings are written correctly."""
        for s_val, binstring in LSM9DS0._TABLE_77.items():
            yield (self._test_bits_written, 'accelerometer', 'self_test',
                   LSM9DS0.ACC_ADDRESS, LSM9DS0.CTRL_REG2_XM, 0b00000110, 1, s_val, binstring)

    # Gyroscope init tests.
    def test_correct_gyro_data_rate_applied(self):
        """Test that Gyroscope settings are written correctly #1."""
        for s_val, binstring in LSM9DS0._TABLE_21_1.items():
            yield (self._test_bits_written, 'gyroscope', 'data_rate',
                   LSM9DS0.GYR_ADDRESS, LSM9DS0.CTRL_REG1_G, 0b11000000, 6, s_val, binstring)

    def test_correct_gyro_bandwidth_applied(self):
        """Test that Gyroscope settings are written correctly."""
        for s_val, binstring in [(0, '00'), (1, '01'), (2, '10'), (3, '11')]:
            yield (self._test_bits_written, 'gyroscope', 'bandwidth_level',
                   LSM9DS0.GYR_ADDRESS, LSM9DS0.CTRL_REG1_G, 0b00110000, 4, s_val, binstring)

    def test_correct_gyro_powerdown_mode(self):
        """Test that Gyroscope settings are written correctly."""
        for s_val, binstring in [(True, '0'), (False, '1')]:
            yield (self._test_bits_written, 'gyroscope', 'powerdown_mode',
                   LSM9DS0.GYR_ADDRESS, LSM9DS0.CTRL_REG1_G, 0b00001000, 3, s_val, binstring)

    def test_correct_gyro_cont_update(self):
        """Test that Gyroscope continuous update written correctly."""
        for s_val, binstring in [(True, '0'), (False, '1')]:
            yield (self._test_bits_written, 'gyroscope', 'continuous_update',
                   LSM9DS0.GYR_ADDRESS, LSM9DS0.CTRL_REG4_G, 0b10000000, 7, s_val, binstring)

    def test_correct_gyro_le(self):
        """Test that Gyroscope settings are written correctly."""
        for s_val, binstring in [(True, '1'), (False, '0')]:
            yield (self._test_bits_written, 'gyroscope', 'little_endian',
                   LSM9DS0.GYR_ADDRESS, LSM9DS0.CTRL_REG4_G, 0b01000000, 6, s_val, binstring)

    def test_correct_gyro_enabled_z(self):
        """Test that Gyroscope settings are written correctly."""
        for s_val, binstring in [(True, '1'), (False, '0')]:
            yield (self._test_bits_written, 'gyroscope', 'enabled_z',
                   LSM9DS0.GYR_ADDRESS, LSM9DS0.CTRL_REG1_G, 0b00000100, 2, s_val, binstring)

    def test_correct_gyro_enabled_y(self):
        """Test that Gyroscope settings are written correctly."""
        for s_val, binstring in [(True, '1'), (False, '0')]:
            yield (self._test_bits_written, 'gyroscope', 'enabled_y',
                   LSM9DS0.GYR_ADDRESS, LSM9DS0.CTRL_REG1_G, 0b00000010, 1, s_val, binstring)

    def test_correct_gyro_enabled_x(self):
        """Test that Gyroscope settings are written correctly."""
        for s_val, binstring in [(True, '1'), (False, '0')]:
            yield (self._test_bits_written, 'gyroscope', 'enabled_x',
                   LSM9DS0.GYR_ADDRESS, LSM9DS0.CTRL_REG1_G, 0b00000001, 0, s_val, binstring)

    def test_correct_gyro_full_scale_applied(self):
        """Test that Gyroscope settings are written correctly."""
        for s_val, binstring in LSM9DS0._TABLE_30.items():
            yield (self._test_bits_written, 'gyroscope', 'full_scale',
                   LSM9DS0.GYR_ADDRESS, LSM9DS0.CTRL_REG4_G, 0b00110000, 4, s_val, binstring)

    def test_correct_gyro_self_test_applied(self):
        """Test that Gyroscope settings are written correctly."""
        for s_val, binstring in LSM9DS0._TABLE_31.items():
            yield (self._test_bits_written, 'gyroscope', 'self_test',
                   LSM9DS0.GYR_ADDRESS, LSM9DS0.CTRL_REG4_G, 0b00000110, 1, s_val, binstring)

    # Magnetometer tests

    def test_correct_mag_data_rate_applied(self):
        """Test that Magnetometer settings are written correctly."""
        for s_val, binstring in LSM9DS0._TABLE_84.items():
            yield (self._test_bits_written, 'magnetometer', 'data_rate',
                   LSM9DS0.MAG_ADDRESS, LSM9DS0.CTRL_REG5_XM, 0b00011100, 2, s_val, binstring)

    def test_correct_mag_enabled_temp(self):
        """Test that Magnetometer settings are written correctly."""
        for s_val, binstring in [(True, '1'), (False, '0')]:
            yield (self._test_bits_written, 'magnetometer', 'enabled_temp',
                   LSM9DS0.MAG_ADDRESS, LSM9DS0.CTRL_REG5_XM, 0b10000000, 7, s_val, binstring)

    def test_correct_mag_full_scale_applied(self):
        """Test that Magnetometer settings are written correctly."""
        for s_val, binstring in LSM9DS0._TABLE_87.items():
            yield (self._test_bits_written, 'magnetometer', 'full_scale',
                   LSM9DS0.MAG_ADDRESS, LSM9DS0.CTRL_REG6_XM, 0b01100000, 5, s_val, binstring)

    def test_correct_mag_sensor_mode_applied(self):
        """Test that Magnetometer settings are written correctly."""
        for s_val, binstring in LSM9DS0._TABLE_90.items():
            yield (self._test_bits_written, 'magnetometer', 'sensor_mode',
                   LSM9DS0.MAG_ADDRESS, LSM9DS0.CTRL_REG7_XM, 0b01100011, 0, s_val, binstring)

    def test_correct_mag_lowpower_mode(self):
        """Test that Magnetometer settings are written correctly."""
        for s_val, binstring in [(True, '1'), (False, '0')]:
            yield (self._test_bits_written, 'magnetometer', 'lowpower_mode',
                   LSM9DS0.MAG_ADDRESS, LSM9DS0.CTRL_REG7_XM, 0b00000100, 2, s_val, binstring)

    def test_correct_mag_high_resolution(self):
        """Test that Magnetometer settings are written correctly."""
        for s_val, binstring in [(True, '11'), (False, '00')]:
            yield (self._test_bits_written, 'magnetometer', 'high_resolution',
                   LSM9DS0.MAG_ADDRESS, LSM9DS0.CTRL_REG5_XM, 0b01100000, 5, s_val, binstring)