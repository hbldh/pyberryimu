#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals
from __future__ import absolute_import

import os
import uuid
import json

from flask import Flask
from flask import render_template, jsonify, request
from concurrent.futures import ProcessPoolExecutor

from pyberryimu.client import BerryIMUClient
from pyberryimu.calibration.standard import StandardCalibration
from pyberryimu.collector import TimedDataCollectionTool
from pyberryimu.container import DataContainer

app = Flask('PyBerryIMU REST API app')

@app.route('/api/1/start', methods=['POST', ])
def collect():
    post_data = request.get_json()
    this_uuid = uuid.uuid4()
    this_file_path = os.path.expanduser('~/pyberryimu_data/{0}.json'.format(this_uuid))
    with open(this_file_path, 'a') as f:
        pass

    def fcn():
        with BerryIMUClient(settings=post_data.get('settings')) as client:
            client.calibration_object = StandardCalibration.load()

            # TODO: Create from settings...
            def _callback():
                return client.read_accelerometer()

            tdct = TimedDataCollectionTool(_callback, post_data.get('frequency'), post_data.get('duration'))
            output = tdct.run()
            dc = DataContainer(start_time=output[0],
                               client_settings=client.get_settings(),
                               calibration_parameters=client.calibration_object.to_json())
            dc.timestamps = output[1]
            dc.accelerometer = output[2]
            dc.save(this_file_path)

    with ProcessPoolExecutor(max_workers=1) as executor:
        future = executor.submit(fcn)

    return jsonify({'uuid': this_uuid})


@app.route('/api/1/collect/<this_uuid>', methods=['GET', ])
def collect(this_uuid):
    this_file_path = os.path.expanduser('~/pyberryimu_data/{0}.json'.format(this_uuid))
    try:
        with open(this_file_path, 'r') as f:
            out = json.load(f)
    except Exception as e:
        out = {'Error': str(e)}
    return jsonify(**out)
