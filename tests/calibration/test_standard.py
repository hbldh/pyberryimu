#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
:mod:`test_standard`
==================

.. module:: test_standard
   :platform: Unix, Windows
   :synopsis: 

.. moduleauthor:: hbldh <henrik.blidh@nedomkull.com>

Created on 2015-07-04, 12:00

"""

from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import

import numpy as np
import numpy.testing as nptest

from ..test_client import create_device
from pyberryimu.calibration.standard import StandardCalibration


class TestAccelerationStandardCalibration(object):
    """Nose Test Suite for Standard Calibration of Accelerometer."""

    def __init__(self):
        """Constructor for TestStandardCalibration"""
        self.test_points_1 = np.array([[-4772.38754098, 154.04459016, -204.39081967],
                                       [3525.0346179, -68.64924886, -34.54604833],
                                       [-658.17681729, -4137.60248854, -140.49377865],
                                       [-564.18562092, 4200.29150327, -130.51895425],
                                       [-543.18289474, 18.14736842, -4184.43026316],
                                       [-696.62532808, 15.70209974, 3910.20734908],
                                       [406.65271419, 18.46827992, -4064.61085677],
                                       [559.45926413, -3989.69513798, -174.71879106],
                                       [597.22629169, -3655.54153041, -1662.83257031],
                                       [1519.02616089, -603.82472204, 3290.58469588]])

        self.test_points_2 = np.array([[-1575.43324607, 58.07787958, -72.69371728],
                                       [1189.53102547, -11.92749837, -23.37687786],
                                       [-212.62989556, -1369.82898172, -48.73498695],
                                       [-183.42717178, 1408.61463096, -33.89745265],
                                       [-162.57253886, 23.43005181, -1394.36722798],
                                       [-216.76963011, 19.37118754, 1300.13822193],
                                       [-809.20208605, 69.1029987, -1251.60104302],
                                       [-1244.03955901, -866.0843061, -67.02594034],
                                       [-1032.3692107, 811.19178082, 699.69602087],
                                       [-538.82617188, -161.6171875, -1337.34895833]])

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_calibration_points_1(self):
        sc = StandardCalibration(verbose=True)
        sc.calibrate_accelerometer_with_stored_points(self.test_points_1)
        np.testing.assert_almost_equal(sc._acc_calibration_errors[-1], 0.0, 2)

    def test_calibration_points_2(self):
        sc = StandardCalibration(verbose=True)
        sc.calibrate_accelerometer_with_stored_points(self.test_points_2)
        np.testing.assert_almost_equal(sc._acc_calibration_errors[-1], 0.0, 2)

    def test_set_datasheet_values(self):
        sc = StandardCalibration(verbose=True)
        client, smbus, mockbus = create_device()
        client.open()
        client.bus._read = mockbus._written
        sc.set_datasheet_values_for_accelerometer(client.get_settings())
        assert np.linalg.norm(sc.acc_scale_factor_matrix - np.eye(3)) > 0.0

class TestGyroscopeStandardCalibration(object):
    """Nose Test Suite for Standard Calibration of Accelerometer."""

    def __init__(self):
        """Constructor for TestStandardCalibration"""
        self.points_1_zero = np.array([1.43261965, -42.49307305, -101.22670025])
        self.test_points_1 = np.array([[-15541.91964846202, -52.51412429378531, -1321.6428123038293],
                                       [15697.744814582024, -3.4299182903834065, 320.561282212445],
                                       [-105.46394984326018, -16087.585579937304, -363.0702194357367],
                                       [182.46071653048398, 16011.094908862351, 61.458830923947204],
                                       [323.26021370207417, -81.91326209930861, -15861.725329981144],
                                       [-255.02514142049026, 90.87177875549969, 15684.67944688875]])

        self.points_2_zero = np.array([10.69647355, -46.17254408, -84.302267])
        self.test_points_2 = np.array([[-11721.697923222153, 75.89616110761486, -252.45689112649464],
                                       [11830.82715273413, -42.7950974230044, 126.3117536140792],
                                       [10.582023884349466, -12080.046511627907, -187.43997485857952],
                                       [78.96922110552764, 11982.507537688442, -37.949120603015075],
                                       [383.20603015075375, 1.160175879396985, -11924.912688442211],
                                       [-272.03077889447235, 52.17902010050251, 11731.529522613066]])

    def test_calibration_points_1_radians(self):
        sc = StandardCalibration(verbose=True)
        sc.calibrate_gyroscope_with_stored_points(
            self.points_1_zero, self.test_points_1, sc.RECORD_PLAYER_45_RPM_IN_RADIANS)
        np.testing.assert_allclose(sc.gyro_bias_vector,
                                   np.array([-0.01581438, 0.01164529, 0.02771216]),
                                   rtol=1e-8, atol=1e-8)
        np.testing.assert_allclose(sc.gyro_scale_factor_vector,
                                   np.array([0.00030169, 0.00029362, 0.00029876]),
                                   rtol=1e-8, atol=1e-8)

    def test_calibration_points_1_degrees(self):
        sc = StandardCalibration(verbose=True)
        sc.calibrate_gyroscope_with_stored_points(
            self.points_1_zero, self.test_points_1, sc.RECORD_PLAYER_45_RPM_IN_DPS)
        np.testing.assert_allclose(sc.gyro_bias_vector,
                                   np.array([-0.01581438, 0.01164529, 0.02771216]) * 57.2957795,
                                   rtol=1e-6, atol=1e-6)
        np.testing.assert_allclose(sc.gyro_scale_factor_vector,
                                   np.array([0.00030169, 0.00029362, 0.00029876]) * 57.2957795,
                                   rtol=1e-6, atol=1e-6)

    def test_calibration_points_2_radians(self):
        sc = StandardCalibration(verbose=True)
        sc.calibrate_gyroscope_with_stored_points(
            self.points_2_zero, self.test_points_2, sc.RECORD_PLAYER_33_3_RPM_IN_RADIANS)
        np.testing.assert_allclose(sc.gyro_bias_vector,
                                   np.array([-0.67834283, 0.79632176, 1.56510056]) * 0.017453293,
                                   rtol=1e-6, atol=1e-6)
        np.testing.assert_allclose(sc.gyro_scale_factor_vector,
                                   np.array([0.01698324, 0.01662334, 0.01690871]) * 0.017453293,
                                   rtol=1e-6, atol=1e-6)

    def test_calibration_points_2_degrees(self):
        sc = StandardCalibration(verbose=True)
        sc.calibrate_gyroscope_with_stored_points(
            self.points_2_zero, self.test_points_2, sc.RECORD_PLAYER_33_3_RPM_IN_DPS)
        np.testing.assert_allclose(sc.gyro_bias_vector,
                                   np.array([-0.67834283, 0.79632176, 1.56510056]),
                                   rtol=1e-6, atol=1e-6)
        np.testing.assert_allclose(sc.gyro_scale_factor_vector,
                                   np.array([0.01698324, 0.01662334, 0.01690871]),
                                   rtol=1e-6, atol=1e-6)

    def test_calibration_points_correspondence_with_estimated_values(self):
        sc = StandardCalibration(verbose=True)
        sc2 = StandardCalibration(verbose=True)
        client, smbus, mockbus = create_device()
        client.open()
        client.bus._read = mockbus._written
        sc.calibrate_gyroscope_with_stored_points(
            self.points_2_zero, self.test_points_2, sc.RECORD_PLAYER_33_3_RPM_IN_DPS)
        sc2.set_datasheet_values_for_gyroscope(client.get_settings())

        def _internal_test_function(untransformed_g, reference, tolerance):
            np.testing.assert_allclose(sc.transform_gyroscope_values(untransformed_g), reference, atol=tolerance)

        yield _internal_test_function, self.points_2_zero, np.array([0.0, 0.0, 0.0]), 0.5

        for index in range(3):
            for side in [-1, 1]:
                k = index * 2 if side == -1 else (index * 2) + 1
                ref = np.zeros((3,), 'float')
                ref[index] = sc.RECORD_PLAYER_33_3_RPM_IN_DPS * side
                yield _internal_test_function, self.test_points_2[k, :], ref, 6.0

    def test_calibration_points_correspondence_with_datasheet_values(self):
        sc = StandardCalibration(verbose=True)
        sc2 = StandardCalibration(verbose=True)
        client, smbus, mockbus = create_device()
        client.open()
        client.bus._read = mockbus._written
        sc.calibrate_gyroscope_with_stored_points(
            self.points_2_zero, self.test_points_2, sc.RECORD_PLAYER_33_3_RPM_IN_DPS)
        sc2.set_datasheet_values_for_gyroscope(client.get_settings())

        def _internal_test_function(untransformed_g, tolerance):
            np.testing.assert_allclose(sc.transform_gyroscope_values(untransformed_g),
                                       sc2.transform_gyroscope_values(untransformed_g - self.points_2_zero),
                                       atol=tolerance)

        for index in range(3):
            for side in [-1, 1]:
                k = index * 2 if side == -1 else (index * 2) + 1
                ref = np.zeros((3,), 'float')
                ref[index] = sc.RECORD_PLAYER_33_3_RPM_IN_DPS * side
                yield _internal_test_function, self.test_points_2[k, :], 15.0

