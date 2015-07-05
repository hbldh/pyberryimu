#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
:mod:`accelerometer`
==================

.. module:: accelerometer
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
import json
import time

import six
import numpy as np

from pyberryimu import version
from pyberryimu.exc import PyBerryIMUError
from pyberryimu.calibration.base import BerryIMUCalibration


class StandardCalibration(BerryIMUCalibration):
    """The Standard Calibration object for the PyBerryIMU."""

    # 45 RPM as radians per second.
    RECORD_PLAYER_45_RPM = (45 / 60) * 2 * np.pi
    # 33 & 1/3 RPM as radians per second.
    RECORD_PLAYER_33_3_RPM = ((33 + (1 / 3)) / 60) * 2 * np.pi

    def __init__(self, verbose=False):
        """Constructor for StandardCalibration"""
        super(StandardCalibration, self).__init__(verbose)

        # Accelerometer calibration parameters.
        self._acc_zero_g = None
        self._acc_sensitivity = None
        self._acc_calibration_points = None
        self._acc_calibration_errors = None

        self.acc_scale_factor_matrix = None
        self.acc_bias_vector = None

        # Gyroscope calibration parameters.
        # TODO: Remove default values here after implementation of gyro calibration.
        self.gyro_bias_vector = np.array([0, 0, 0], 'float')
        self.gyro_scale_factor_vector = np.array([1, 1, 1], 'float')

        self.__mid_v = 2 ** 15
        self.__max_v = (2 ** 16) - 1

    @classmethod
    def load(cls, doc_path=os.path.expanduser('~/.pyberryimu')):
        with open(doc_path, 'rt') as f:
            doc = json.load(f)

        out = cls()

        # Transfer BerryIMU settings.
        out.berryimu_settings = doc.get('pyberryimu_version', version)
        out.pyberryimu_version = doc.get('BerryIMU_settings', {})

        # Parse accelerometer calibration values.
        acc_doc = doc.get('accelerometer', {})
        out.acc_scale_factor_matrix = np.reshape(
            np.array(acc_doc.get('scale_factor', np.eye(3).flatten()), 'float'), (3, 3))
        out.acc_bias_vector = np.array(acc_doc.get('bias', [0, 0, 0]), 'float')

        # Parse gyroscope calibration values.
        gyro_doc = doc.get('gyroscope', {})
        out.gyro_bias_vector = np.array(gyro_doc.get('bias', [0, 0, 0]), 'float')
        out.gyro_scale_factor_vector = np.array(
            gyro_doc.get('scale_factor', [1, 1, 1]), 'float')

        return out

    def save(self, save_path=os.path.expanduser('~/.pyberryimu')):
        try:
            doc = self.to_json()
        except Exception as e:
            raise PyBerryIMUError("Could not save calibration data to file: {0}".format(e))
        with open(save_path, 'wt') as f:
            json.dump(doc, f, indent=4)

    def to_json(self):
        doc = super(StandardCalibration, self).to_json()
        doc.update({
            'accelerometer': {
                'scale_factor': self.acc_scale_factor_matrix.flatten().tolist(),
                'bias': self.acc_bias_vector.tolist()
            },
            'gyro': {
                'scale_factor': self.gyro_scale_factor_vector.tolist(),
                'bias': self.gyro_bias_vector.tolist(),
            }
        })
        return doc

    # Help methods

    def acc_to_ratio(self, x):
        return (x + self.__mid_v) / self.__max_v

    @staticmethod
    def rpm_to_rads_per_sec(rpm_value):
        return (rpm_value / 60) * 2 * np.pi

    # Accelerometer calibration methods

    def calibrate_accelerometer(self, client, **kwargs):
        """Perform calibration of accelerometer.

        Computes the Zero G levels, Sensitivity, Scale factor Matrix and the
        bias vector of a MEMS accelerometer.

        The procedure exploits the fact that, in static conditions, the
        modulus of the accelerometer output vector matches that of the
        gravity acceleration. The calibration model incorporates the bias
        and scale factor for each axis and the cross-axis symmetrical
        factors. The parameters are computed through Gauss-Newton
        nonlinear optimization.

        The mathematical model used is  A = M(V - B)
        where M and B are scale factor matrix and bias vector respectively.
        Note that the vector V has elements in [0, 1.0] representing the
        normed output value of the sensor. This is calculated by
        `(a_raw + (2 ** 15)) / ((2 ** 16) - 1)`.

        M = [ Mxx Mxy Mxz; Myx Myy Myz; Mzx Mzy Mzz ]
        where  Mxy = Myx; Myz = Mzy; Mxz = Mzx;
        B = [ Bx; By; Bz ]

        The diagonal elements of M represent the scale factors along the
        three axes, whereas the other elements of M are called cross-axis
        factors. These terms allow describing both the axes’ misalignment
        and the crosstalk effect between different channels caused
        by the sensor electronics. In an ideal world, M = 1; B = 0

        First, six points of +/-1 g on each axis is recorded. From these readings
        a first estimate of zero G offset and primary axis scale factors is obtained.
        To find the zero values of your own accelerometer, note the max and
        minimum of the ADC values for each axis and use the following formula:
        Zero_x = (Max_x - Min_x)/2; ...
        To find the Sensitivity use the following formula:
        Sensitivity_x = 2 / (Max_x - Min_x); ...

        Reference:
        Iuri Frosio, Federico Pedersini, N. Alberto Borghese
        "Autocalibration of MEMS Accelerometers"
        IEEE TRANSACTIONS ON INSTRUMENTATION AND MEASUREMENT, VOL. 58, NO. 6, JUNE 2009

        This is a Python reimplementation of the Matlab routines found at
        `Matlab File Central <http://se.mathworks.com/matlabcentral/fileexchange/
        33252-mems-accelerometer-calibration-using-gauss-newton-method>`_.

        :param client: The BerryIMU communication client.
        :type client: :py:class:`pyberryimu.client.BerryIMUClient`

        """
        if self.acc_scale_factor_matrix is not None:
            raise PyBerryIMUError('This object has already been calibrated!')

        self.berryimu_settings = client.get_settings()
        points = self._do_six_point_one_g_calibration(client)
        points += self._add_additional_points(client)
        self._acc_calibration_points = np.array(points)
        self._perform_accelerometer_calibration_optimisation(
            self.acc_to_ratio(self._acc_calibration_points))
      
    def calibrate_accelerometer_with_stored_points(self, points):
        """Perform calibration of accelerometer with stored points.

        :param points: Calibration points recorded earlier.
        :type points: :py:class:`numpy.ndarray`

        """
        if self.acc_scale_factor_matrix is not None:
            raise PyBerryIMUError('This object has already been calibrated!')

        self._acc_zero_g = np.zeros((3, ), 'float')
        self._acc_sensitivity = np.zeros((3, ), 'float')
        for index in six.moves.range(3):
            this_axis_points = []
            for side in [0, 1]:
                this_axis_points.append(self.acc_to_ratio(points[index * 2 + side, index]))

            v_max, v_min = max(this_axis_points), min(this_axis_points)
            self._acc_zero_g[index] = (v_max + v_min) / 2
            v_max = v_max - self._acc_zero_g[index]
            v_min = v_min - self._acc_zero_g[index]
            self._acc_sensitivity[index] = 2 / (v_max - v_min)
        points = self.acc_to_ratio(np.array(points))
        self._perform_accelerometer_calibration_optimisation(points)

    def _do_six_point_one_g_calibration(self, client):
        """Perform six recording of +/- 1g on each accelerometer axis.

        :param client: The BerryIMU communication client.
        :type client: :py:class:`pyberryimu.client.BerryIMUClient`
        :return: List of points added.
        :rtype: list

        """
        points = []
        self._acc_zero_g = np.zeros((3, ), 'float')
        self._acc_sensitivity = np.zeros((3, ), 'float')

        # Method for polling until desired axis is oriented as requested.
        def _wait_for_compliance():
            keep_waiting = 10
            while keep_waiting > 0:
                a = client.read_accelerometer()
                norm_a = np.linalg.norm(a)
                norm_diff = np.abs(np.abs(a[index]) - norm_a) / norm_a

                if norm_diff < 0.05 and cmp(a[index], 0) == side:
                    keep_waiting -= 1
                else:
                    keep_waiting = 10
                time.sleep(0.1)

        axes_names = ['x', 'y', 'z']
        for index in six.moves.range(3):
            this_axis_points = []
            for side in [-1, 1]:
                print('Position BerryIMU {0} axis {1}...'.format(
                    axes_names[index], 'downwards' if side < 0 else 'upwards'))
                _wait_for_compliance()
                raw_input('Correct orientation. Start calibration of BerryIMU {0} '
                          'axis {1} ({2}) by pressing Enter.'.format(axes_names[index],
                                                                     'downwards' if side < 0 else 'upwards',
                                                                     client.read_accelerometer()))
                acc_values = []
                t = time.time()
                while (time.time() - t) < 5:
                    acc_values.append(client.read_accelerometer())

                points.append(np.mean(acc_values, axis=0).tolist())
                this_axis_points.append(self.acc_to_ratio(points[-1][index]))

            v_max, v_min = max(this_axis_points), min(this_axis_points)
            self._acc_zero_g[index] = (v_max + v_min) / 2
            v_max = v_max - self._acc_zero_g[index]
            v_min = v_min - self._acc_zero_g[index]
            self._acc_sensitivity[index] = 2 / (v_max - v_min)

        return points

    def _add_additional_points(self, client):
        """Add more calibration points.

        Six calibration points have already been recorded in the six direction zero G/sensitivity
        part of the calibration. At least three more has to be added to be able to perform optimisation
        for scale factors and bias.

        :param client: The BerryIMU communication client.
        :type client: :py:class:`pyberryimu.client.BerryIMUClient`
        :return: List of points added.
        :rtype: list

        """
        points = []
        while True:
            ch = raw_input('At least {0} more points are required. '
                           'Add another calibration point? (y / n) '.format(max([3 - len(points), 0])))
            if ch == 'y':
                raw_input('Make sure BerryIMU is static and then start gathering data by pressing Enter.')
                acc_values = []
                t = time.time()
                while (time.time() - t) < 5:
                    acc_values.append(client.read_accelerometer())
                points.append(np.mean(acc_values, axis=0).tolist())
            elif ch == 'n':
                break
            else:
                pass
        return points

    def _perform_accelerometer_calibration_optimisation(self, points):
        """Perform the Gauss-Newton optimisation for parameters.

        :param points: The calibration points recorded.
        :type points: :py:class:`numpy.ndarray`

        """
        nbr_points = len(points)
        if nbr_points < 9:
            raise ValueError('Need at least 9 Measurements for the calibration procedure!')

        # Optimisation error function.
        def error_function(M_mat, b_vec, y):
            return np.sum((M_mat.dot((y - b_vec)) ** 2)) - 1

        # Method for calculating the Jacobian.
        def _jacobian(M_mat, b_vec, point):
            # TODO: Clean up Jacobian calculation code. Make it more concise.
            jac = np.zeros((9, ), 'float')

            jac[0] = 2 * (b_vec[0] - point[0]) * (
                M_mat[0, 0] * (b_vec[0] - point[0]) + M_mat[0, 1] * (b_vec[1] - point[1]) + M_mat[0, 2] * (
                    b_vec[2] - point[2]))
            jac[1] = 2 * (b_vec[1] - point[1]) * (
                M_mat[0, 0] * (b_vec[0] - point[0]) + M_mat[0, 1] * (b_vec[1] - point[1]) + M_mat[0, 2] * (
                    b_vec[2] - point[2])) + 2 * (b_vec[0] - point[0]) * (
                M_mat[0, 1] * (b_vec[0] - point[0]) + M_mat[1, 1] * (b_vec[1] - point[1]) + M_mat[1, 2] * (
                    b_vec[2] - point[2]))
            jac[2] = 2 * (b_vec[0] - point[0]) * (
                M_mat[0, 2] * (b_vec[0] - point[0]) + M_mat[1, 2] * (b_vec[1] - point[1]) + M_mat[2, 2] * (
                    b_vec[2] - point[2])) + 2 * (b_vec[2] - point[2]) * (
                M_mat[0, 0] * (b_vec[0] - point[0]) + M_mat[0, 1] * (b_vec[1] - point[1]) + M_mat[0, 2] * (
                    b_vec[2] - point[2]))
            jac[3] = 2 * (b_vec[1] - point[1]) * (
                M_mat[0, 1] * (b_vec[0] - point[0]) + M_mat[1, 1] * (b_vec[1] - point[1]) + M_mat[1, 2] * (
                    b_vec[2] - point[2]))
            jac[4] = 2 * (b_vec[1] - point[1]) * (
                M_mat[0, 2] * (b_vec[0] - point[0]) + M_mat[1, 2] * (b_vec[1] - point[1]) + M_mat[2, 2] * (
                    b_vec[2] - point[2])) + 2 * (b_vec[2] - point[2]) * (
                M_mat[0, 1] * (b_vec[0] - point[0]) + M_mat[1, 1] * (b_vec[1] - point[1]) + M_mat[1, 2] * (
                    b_vec[2] - point[2]))
            jac[5] = 2 * (b_vec[2] - point[2]) * (
                M_mat[0, 2] * (b_vec[0] - point[0]) + M_mat[1, 2] * (b_vec[1] - point[1]) + M_mat[2, 2] * (
                    b_vec[2] - point[2]))
            jac[6] = 2 * M_mat[0, 0] * (
                M_mat[0, 0] * (b_vec[0] - point[0]) + M_mat[0, 1] * (b_vec[1] - point[1]) + M_mat[0, 2] * (
                    b_vec[2] - point[2])) + 2 * M_mat[0, 1] * (
                M_mat[0, 1] * (b_vec[0] - point[0]) + M_mat[1, 1] * (b_vec[1] - point[1]) + M_mat[1, 2] * (
                    b_vec[2] - point[2])) + 2 * M_mat[0, 2] * (
                M_mat[0, 2] * (b_vec[0] - point[0]) + M_mat[1, 2] * (b_vec[1] - point[1]) + M_mat[2, 2] * (
                    b_vec[2] - point[2]))
            jac[7] = 2 * M_mat[0, 1] * (
                M_mat[0, 0] * (b_vec[0] - point[0]) + M_mat[0, 1] * (b_vec[1] - point[1]) + M_mat[0, 2] * (
                    b_vec[2] - point[2])) + 2 * M_mat[1, 1] * (
                M_mat[0, 1] * (b_vec[0] - point[0]) + M_mat[1, 1] * (b_vec[1] - point[1]) + M_mat[1, 2] * (
                    b_vec[2] - point[2])) + 2 * M_mat[1, 2] * (
                M_mat[0, 2] * (b_vec[0] - point[0]) + M_mat[1, 2] * (b_vec[1] - point[1]) + M_mat[2, 2] * (
                    b_vec[2] - point[2]))
            jac[8] = 2 * M_mat[0, 2] * (
                M_mat[0, 0] * (b_vec[0] - point[0]) + M_mat[0, 1] * (b_vec[1] - point[1]) + M_mat[0, 2] * (
                    b_vec[2] - point[2])) + 2 * M_mat[1, 2] * (
                M_mat[0, 1] * (b_vec[0] - point[0]) + M_mat[1, 1] * (b_vec[1] - point[1]) + M_mat[1, 2] * (
                    b_vec[2] - point[2])) + 2 * M_mat[2, 2] * (
                M_mat[0, 2] * (b_vec[0] - point[0]) + M_mat[1, 2] * (b_vec[1] - point[1]) + M_mat[2, 2] * (
                    b_vec[2] - point[2]))

            return jac

        # Convenience method for moving between optimisation vector and correct lin.alg. formulation.
        def optvec_to_M_and_b(v):
            return np.array([[v[0], v[1], v[2]], [v[1], v[3], v[4]], [v[2], v[4], v[5]]]), v[6:].copy()

        gain = 1  # Damping Gain - Start with 1
        damping = 0.01    # Damping parameter - has to be less than 1.
        tolerance = 1e-12
        R_prior = 100000
        self._acc_calibration_errors = []
        nbr_iterations = 200

        # Initial Guess values of M and b.
        x = np.array([self._acc_sensitivity[0], 0.0, 0.0,
                      self._acc_sensitivity[1], 0.0, self._acc_sensitivity[2],
                      self._acc_zero_g[0], self._acc_zero_g[1], self._acc_zero_g[2]])
        last_x = x.copy()
        # Residuals vector
        R = np.zeros((nbr_points, ), 'float')
        # Jacobian matrix
        J = np.zeros((nbr_points, 9), 'float')

        for n in six.moves.range(nbr_iterations):
            # Calculate the Jacobian at every iteration.
            M, b = optvec_to_M_and_b(x)            
            for i in six.moves.range(nbr_points):
                R[i] = error_function(M, b, points[i, :])
                J[i, :] = _jacobian(M, b, points[i, :])

            # Calculate Hessian, Gain matrix and apply it to solution vector.
            H = np.linalg.inv(J.T.dot(J))
            D = J.T.dot(R).T
            x -= gain * (D.dot(H)).T
            R_post = np.linalg.norm(R)
            if self._verbose:
                print("{0}: {1} ({2})".format(n, R_post, ", ".join(["{0:0.9g}".format(v) for v in x])))

            # This is to make sure that the error is decreasing with every iteration.
            if R_post <= R_prior:
                gain -= damping * gain
            else:
                gain *= damping

            # Iterations are stopped when the following convergence criteria is satisfied.
            if abs(max(2 * (x - last_x) / (x + last_x))) <= tolerance:
                self.acc_scale_factor_matrix, self.acc_bias_vector = optvec_to_M_and_b(x)
                break

            last_x = x.copy()
            R_prior = R_post
            self._acc_calibration_errors.append(R_post)

    # Gyroscope calibration methods

    def calibrate_gyroscope(self, client, radians_per_sec=None):
        """Linear Regression model fitting for SI unit conversion of gyroscope data.

        Requires a plane surface rotating with a fixed and known speed
        (e.g. a vinyl record player) on which the BerryIMU can be placed.
        The user then positions the BerryIMU so that all rotation is captured
        by one sensor axis. After six such measurements and a static reading,
        the scale and offset values are determined by linear regression, s.t.
        ``g_calib = scale * g_raw + bias``.

        Note that no cross-talk terms are present in this simple gyroscope calibration.

        :param client: The BerryIMU communication client.
        :type client: :py:class:`pyberryimu.client.BerryIMUClient`
        :param radians_per_sec: Rotation speed of the calibration plane.
        :type radians_per_sec: float

        """
        if radians_per_sec is None:
            raise PyBerryIMUError("A fixed rotation value in radians per seconds "
                                  "must be given. See docstring.")

        points = []
        gyro_zero = np.zeros((3, ), 'float')

        gyro_scale = np.zeros((3, ), 'float')
        gyro_bias = np.zeros((3, ), 'float')

        # Method for polling until desired axis is oriented as requested.
        def _wait_for_compliance():
            keep_waiting = 4
            while keep_waiting > 0:
                g = np.array(client.read_gyroscope()) - gyro_zero
                print(g)
                norm_g = np.linalg.norm(g)
                norm_diff = np.abs(np.abs(g[index]) - norm_g) / norm_g

                if norm_diff < 0.01 and cmp(g[index], 0) == side:
                    keep_waiting -= 1
                else:
                    keep_waiting = 4
                time.sleep(0.25)

        # Record the zero level of the gyros.
        raw_input('Let the BerryIMU be completely still. Record Zero values by pressing Enter.')
        gyro_values = []
        t = time.time()
        while (time.time() - t) < 5:
            gyro_values.append(client.read_gyroscope())
        gyro_zero = np.mean(gyro_values, axis=0)

        axes_names = ['x', 'y', 'z']
        for index in six.moves.range(3):
            this_axis_points = []
            for side in [-1, 1]:
                print("Position the BerryIMU so that all gyro output is "
                      "{0} on the {1} gyro axis...".format(
                          'negative' if side < 0 else 'positive', axes_names[index]))
                _wait_for_compliance()
                raw_input('Correct orientation. Start calibration of BerryIMU Gyro {0} '
                          'axis {1} ({2}) by pressing Enter.'.format(axes_names[index],
                                                                     'negative' if side < 0 else 'positive',
                                                                     client.read_gyroscope()))
                gyro_values = []
                t = time.time()
                while (time.time() - t) < 5:
                    gyro_values.append(client.read_gyroscope())

                points.append(np.mean(gyro_values, axis=0).tolist())
                this_axis_points.append(self.acc_to_ratio(points[-1][index]))

                x = [min(this_axis_points), gyro_zero[index], max(this_axis_points)]
                y = [-radians_per_sec, 0, radians_per_sec]
                gyro_scale[index], gyro_bias[index] = np.polyfit(x, y, 1)

        self.gyro_bias_vector = gyro_bias
        self.gyro_scale_factor_vector = gyro_scale

    def transform_accelerometer_values(self, acc_values):
        # Normalize and then apply the calibration scale matrix and bias.
        converted_g_values = self.acc_scale_factor_matrix.dot(
            self.acc_to_ratio(np.array(acc_values)) - self.acc_bias_vector)
        return tuple(converted_g_values.tolist())

    def transform_gyroscope_values(self, gyro_values):
        return tuple((self.gyro_scale_factor_vector * gyro_values) +
                     self.gyro_bias_vector.tolist())

    def transform_magnetometer_values(self, mag_values):
        # TODO: Study magnetometer calibration. Needed? Zero level is already taken care of.
        return mag_values
