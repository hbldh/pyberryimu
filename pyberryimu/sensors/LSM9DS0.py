#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
:mod:`LSM9DS0`
==================

.. module:: LSM9DS0
   :platform: Unix
   :synopsis: Address for communicating with the LSM9DS0 sensor.
              See `LSM9DS0 data sheet
              <http://ozzmaker.com/wp-content/uploads/2014/12/LSM9DS0.pdf>`_
              for more details.

"""

MAG_ADDRESS = 0x1E
ACC_ADDRESS = 0x1E
GYR_ADDRESS = 0x6A

# LSM9DS0 Gyro Registers
WHO_AM_I_G = 0x0F
CTRL_REG1_G = 0x20
CTRL_REG2_G = 0x21
CTRL_REG3_G = 0x22
CTRL_REG4_G = 0x23
CTRL_REG5_G = 0x24
REFERENCE_G = 0x25
STATUS_REG_G = 0x27
OUT_X_L_G = 0x28
OUT_X_H_G = 0x29
OUT_Y_L_G = 0x2A
OUT_Y_H_G = 0x2B
OUT_Z_L_G = 0x2C
OUT_Z_H_G = 0x2D
FIFO_CTRL_REG_G = 0x2E
FIFO_SRC_REG_G = 0x2F
INT1_CFG_G = 0x30
INT1_SRC_G = 0x31
INT1_THS_XH_G = 0x32
INT1_THS_XL_G = 0x33
INT1_THS_YH_G = 0x34
INT1_THS_YL_G = 0x35
INT1_THS_ZH_G = 0x36
INT1_THS_ZL_G = 0x37
INT1_DURATION_G = 0x38

# LSM9DS0 Accel and Magneto Registers
OUT_TEMP_L_XM = 0x05
OUT_TEMP_H_XM = 0x06
STATUS_REG_M = 0x07
OUT_X_L_M = 0x08
OUT_X_H_M = 0x09
OUT_Y_L_M = 0x0A
OUT_Y_H_M = 0x0B
OUT_Z_L_M = 0x0C
OUT_Z_H_M = 0x0D
WHO_AM_I_XM = 0x0F
INT_CTRL_REG_M = 0x12
INT_SRC_REG_M = 0x13
INT_THS_L_M = 0x14
INT_THS_H_M = 0x15
OFFSET_X_L_M = 0x16
OFFSET_X_H_M = 0x17
OFFSET_Y_L_M = 0x18
OFFSET_Y_H_M = 0x19
OFFSET_Z_L_M = 0x1A
OFFSET_Z_H_M = 0x1B
REFERENCE_X = 0x1C
REFERENCE_Y = 0x1D
REFERENCE_Z = 0x1E
CTRL_REG0_XM = 0x1F
CTRL_REG1_XM = 0x20
CTRL_REG2_XM = 0x21
CTRL_REG3_XM = 0x22
CTRL_REG4_XM = 0x23
CTRL_REG5_XM = 0x24
CTRL_REG6_XM = 0x25
CTRL_REG7_XM = 0x26
STATUS_REG_A = 0x27
OUT_X_L_A = 0x28
OUT_X_H_A = 0x29
OUT_Y_L_A = 0x2A
OUT_Y_H_A = 0x2B
OUT_Z_L_A = 0x2C
OUT_Z_H_A = 0x2D
FIFO_CTRL_REG = 0x2E
FIFO_SRC_REG = 0x2F
INT_GEN_1_REG = 0x30
INT_GEN_1_SRC = 0x31
INT_GEN_1_THS = 0x32
INT_GEN_1_DURATION = 0x33
INT_GEN_2_REG = 0x34
INT_GEN_2_SRC = 0x35
INT_GEN_2_THS = 0x36
INT_GEN_2_DURATION = 0x37
CLICK_CFG = 0x38
CLICK_SRC = 0x39
CLICK_THS = 0x3A
TIME_LIMIT = 0x3B
TIME_LATENCY = 0x3C
TIME_WINDOW = 0x3D

# Accelerometer settings

# Accelerometer data refresh rates
_TABLE_72 = {
    0: '0000',
    3.125: '0001',
    6.25: '0010',
    12.5: '0011',
    25: '0100',
    50: '0100',
    100: '0110',
    200: '0111',
    400: '1000',
    800: '1001',
    1600: '1010',
}

# Accelerometer anti-alias filter bandwidth
_TABLE_75 = {
    773: '00',
    194: '01',
    362: '10',
    50: '11',
}

# Accelerometer full-scale selection
_TABLE_76 = {
    2: '000',
    4: '001',
    6: '010',
    8: '011',
    16: '100',
}

# Accelerometer self-test mode
_TABLE_77 = {
    0: '00',    # Normal mode
    1: '01',    # Positive sign self-test
    -1: '10',   # Negative sign self-test
    'X': '11',  # Not allowed
}


def get_accelerometer_data_rate_bits(data_rate):
    return _TABLE_72.get(data_rate, '0000')


def get_accelerometer_anti_alias_filter_bits(anti_alias):
    return _TABLE_75.get(anti_alias, '00')


def get_accelerometer_full_scale_bits(full_scale):
    return _TABLE_75.get(full_scale, '100')


def get_accelerometer_self_test_bits(self_test):
    return _TABLE_76.get(self_test, '00')

# Gyroscope settings

# Gyroscope output data rate selection
_TABLE_21_1 = {
    95: '00',
    190: '01',
    380: '10',
    760: '11',
}

# Gyroscope full-scale selection.
_TABLE_30 = {
    245: '00',
    500: '01',
    2000: '11',
}


# Gyroscope self-test mode
_TABLE_31 = {
    0: '00',   # Normal mode
    1: '01',   # Self-test 0
    -1: '11',  # Self-test 1
}


def get_gyroscope_data_rate_bits(data_rate):
    return _TABLE_21_1.get(data_rate, '00')


def get_gyroscope_bandwidth_bits(level):
    if level is None:
        return '00'
    else:
        return ['00', '01', '10', '11'][level]


def get_gyroscope_full_scale_bits(full_scale):
    return _TABLE_30.get(full_scale, '11')


def get_gyroscope_self_test_bits(self_test):
    return _TABLE_31.get(self_test, '00')

# Magnetometer settings

# Magnetometer output data rate selection
_TABLE_84 = {
    3.125: '000',
    6.25: '001',
    12.5: '010',
    25: '011',
    50: '100',
    100: '110',
}

# Magnetometer full-scale selection.
_TABLE_87 = {
    2: '00',
    4: '01',
    8: '10',
    12: '11'
}


# Magnetometer sensor mode selection.
_TABLE_90 = {
    0: '00',   # Continuous-conversion mode
    1: '01',   # Single-conversion mode
    -1: '11',  # Power-down mode
}


def get_magnetometer_data_rate_bits(data_rate):
    return _TABLE_84.get(data_rate, '100')


def get_magnetometer_full_scale_bits(full_scale):
    return _TABLE_87.get(full_scale, '11')


def get_magnetometer_sensor_mode_bits(sensor_mode):
    return _TABLE_90.get(sensor_mode, '00')
