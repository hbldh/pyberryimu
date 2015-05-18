#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
:mod:`BMP180`
==================

.. module:: BMP180
   :platform: Unix
   :synopsis: Address for communicating with the BMP180 sensor.

"""

# General address
ADDRESS = 0x77

# Registers
CALIB_DATA_REG = 0xAA
CHIP_ID_REG = 0xD0
WRITE_REG = 0xF4
READ_REG = 0xF6
