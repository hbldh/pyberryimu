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


class TimedDataCollectionTool(object):
    """"""

    def __init__(self, callback_function, frequency, duration):
        """Constructor for TimedDataCollectionTool"""
        self.callback = callback_function
        self.frequency = frequency
        self.duration = duration

    def run(self):
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
            data.append(self.callback())
            if (t - start_t) > self.duration:
                break

        return start_dt, timestamps, data


def main():

    def sleep_slightly():
        time.sleep(0.001)

    tdct = TimedDataCollectionTool(sleep_slightly, 200, 2)
    out = tdct.run()
    print(out[1])

if __name__ == "__main__":
    main()