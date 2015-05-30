#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
:mod:`kalman`
==================

.. module:: kalman
   :platform: Unix, Windows
   :synopsis:

.. moduleauthor:: hbldh <henrik.blidh@nedomkull.com>

Created on 2015-05-30, 16:24

"""

from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import

import time

import numpy as np


class KalmanFilter(object):
    """A simple Kalman Filter implementation, straight from definition.

    N.B. that this filter is NOT an efficient implementation of a Kalman Filter,
    but included more as a first test to see what it might do to the BerryIMU data.

    Consider looking at the :py:module:`pykalman` module for a better implementation.

    """

    def __init__(self, transition_matrix, transition_covariance,
                 observation_matrix, observation_covariance):

        self.transition_matrix = transition_matrix
        self.transition_covariance = transition_covariance
        self.observation_matrix = observation_matrix
        self.observation_covariance = observation_covariance

        self.X = None
        self.P = None

    def set_initial_state(self, initial_state, initial_covariance):
        self.X = initial_state
        self.P = initial_covariance

    def kf_predict(self, X, P):
        # Predict New State, X_k = A * X_{k-1}
        X = self.transition_matrix.dot(X)
        # Predict New Covariance, P_k = A * P_{k-1} * A^T + Sigma_X
        P = self.transition_matrix.dot(P.dot(self.transition_matrix.T)) + \
            self.transition_covariance
        return X, P

    def kf_update(self, X, P, Y):
        # The Mean of predictive distribution of Y
        IM = self.observation_matrix.dot(X)
        # The Covariance or predictive mean of Y
        IS = self.observation_matrix.dot(P.dot(self.observation_matrix.T)) + \
            self.observation_covariance
        IS_inv = np.linalg.inv(IS)
        # Kalman Gain
        K = P.dot(self.observation_matrix.T.dot(IS_inv))
        # Prediction correction with observations.
        X = X + K.dot(Y - IM)
        # Covariance correction by Kalman gain and residual.
        P = P - K.dot(IS.dot(K.T))

        #LH = self.gauss_pdf(Y, IM, IS, IS_inv)
        return X, P, K, IM, IS, #LH

    def gauss_pdf(self, X, M, S, S_inv):
        if M.shape[0] == 1:
            DX = X - np.tile(M, X.shape()[1])
            E = 0.5 * sum(DX * (S_inv.dot(DX)), axis=0)
            E = (E + 0.5 * M.shape()[0] * np.log(2 * np.pi) +
                 0.5 * np.log(np.linalg.det(S)))
            P = np.exp(-E)
        elif X.shape[0] == 1:
            DX = np.tile(X, M.shape()[1]) - M
            E = 0.5 * sum(DX * (S_inv.dot(DX)), axis=0)
            E = (E + 0.5 * M.shape()[0] * np.log(2 * np.pi) +
                 0.5 * np.log(np.linalg.det(S)))
            P = np.exp(-E)
        else:
            DX = X - M
            E = 0.5 * DX.T.dot(S_inv.dot(DX))
            E = (E + 0.5 * M.shape[0] * np.log(2 * np.pi) +
                 0.5 * np.log(np.linalg.det(S)))
            P = np.exp(-E)
        return P, E


def main():
    import os
    observations = np.load(os.path.expanduser('~/Dropbox/NMM/ABB_lp.npy'))

    # Transition matrix
    # A = np.array([
    #     [1, 1, 0, 0, 0, 0, 0],
    #     [0, 1, 0, 0, 0, 0, 0],
    #     [0, 0, 1, 1, 0, 0, 0],
    #     [0, 0, 0, 1, 0, 0, 0],
    #     [0, 0, 0, 0, 1, 1, 0],
    #     [0, 0, 0, 0, 0, 1, 0],
    #     [0, 1, 0, 1, 0, 1, 1]], 'float')
    A = np.array([
        [1, 0.5],
        [0, 1]], 'float')

    transition_covariance = 2.0 * np.eye(A.shape[0])

    # Observation matrix
    # C = np.array([
    #     [1, 0, 0, 0, 0, 0, 0],
    #     [0, 0, 1, 0, 0, 0, 0],
    #     [0, 0, 0, 0, 1, 0, 0],
    #     [0, 0, 0, 0, 0, 0, 1]], 'float')
    C = np.array([
        [1, 0]], 'float')

    observation_covariance = 2.0 * np.eye(C.shape[0])

    # initial_state_mean = [0.33, 0.0, 0.33, 0.0, 0.33, 0.0, 1.0]
    # initial_state_covariance = 0.05 * np.eye(A.shape[0])
    initial_state_mean = np.array([150.0, 1.0])
    initial_state_covariance = np.array([[0.2, 0.2], [0.2, 0.2]])*200

    kf = KalmanFilter(A, transition_covariance, C, observation_covariance)
    kf.set_initial_state(initial_state_mean, initial_state_covariance)

    try:
        import pykalman

        print("KF - pykalman")
        t = time.time()
        kf2 = pykalman.KalmanFilter(A, C,
                                    transition_covariance, observation_covariance,
                                    initial_state_mean=initial_state_mean,
                                    initial_state_covariance=initial_state_covariance)
        filtered_state_estimates = kf2.filter(observations)[0]
        print("Time taken: {0} s".format(t - time.time()))

        print('EM')
        t = time.time()
        kf3 = pykalman.KalmanFilter(em_vars=['transition_covariance', 'observation_covariance'])
        print(kf3.em(observations, n_iter=5))
        print(kf3.transition_covariance)
        print(kf3.observation_covariance)
        print("Time taken: {0} s".format(t - time.time()))
    except ImportError:
        pass

    print("Kalman Filter")
    t = time.time()
    States = []
    X = initial_state_mean
    P = initial_state_covariance
    for o in observations:
        X, P = kf.kf_predict(X, P)
        X, P, K, IM, IS = kf.kf_update(X, P, o)
        States.append(X)

    States = np.array(States)
    print("Time taken: {0} s".format(t - time.time()))

    try:
        # Draw estimates
        import matplotlib.pyplot as pl
        pl.figure()
        ax = pl.subplot2grid((2,2), (0,0), colspan=2)
        lines_true = pl.plot(np.array(observations), color='b')
        lines_filt1 = pl.plot(np.array(filtered_state_estimates[:, 0]), color='r')
        lines_filt2 = pl.plot(np.array(States[:, 0]), 'm--')

        ax3 = pl.subplot2grid((2,2), (1,0), sharex=ax, colspan=2)
        lines_filt1 = pl.plot(np.array(States[:, 1]), color='b')
        lines_filt2 = pl.plot(np.array(filtered_state_estimates[:, 1]), 'r--')

        pl.show()
    except ImportError:
        pass

if __name__ == '__main__':
    main()
