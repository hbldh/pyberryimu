# PyBerryIMU

I2C communications lib for using [BerryIMU]
(http://ozzmaker.com/product/berryimu-accelerometer-gyroscope-magnetometer-barometricaltitude-sensor/).

**This library uses and includes several snippets of code from the 
[Official BerryIMU repository](http://github.com/mwilliams03/BerryIMU).** 

## Installation

#### Required Raspbian packages
- build-essential
- libi2c-dev
- i2c-tools
- python-dev
- libffi-dev

After the package dependencies above are installed, PyBerryIMU can be installed with pip: 

    pip install https://github.com/hbldh/pyberryimu

This library uses [smbus-cffi](https://github.com/bivab/smbus-cffi) for communication over i2c.

## Usage

### Basic use

The BerryIMU can be interfaced with by using the `BerryIMUClient`:

```python
from pyberryimu.client import BerryIMUClient

with BerryIMUClient(bus=1) as c:
    acc = c.read_accelerometer()
    gyro = c.read_gyroscope()
    mag = c.read_magnetometer()
    pr = c.read_pressure()
    temp = c.read_temperature()
```

This returns raw readings from the BerryIMU regarding acceleration, 
angular velocity and magnetic fields; it requires calibration to be useful.
The pressure and temperature are already converted to SI units.

#### Recorder to obtain data for offline analysis

A simple tool for recording data from the BerryIMU to have for offline analysis
is also included in the module.

```python
import os
from pyberryimu.client import BerryIMUClient
from pyberryimu.recorder import BerryIMURecorder

with BerryIMUClient() as c:
    brec = BerryIMURecorder(c, frequency=100, duration=10)
    data_container = brec.record(acc=True, gyro=True, mag=True, pres=False, temp=False)
    data_container.save(os.path.expanduser('~/pyberryimu_rec_test.json'))
```

The data is then stored in a highly non-optimized way as a JSON document that can be loaded as such:

```python
import os
from pyberryimu.container import BerryIMUDataContainer

data_container = BerryIMUDataContainer.load(os.path.expanduser('~/pyberryimu_rec_test.json'))
```

See the example in [pyberryimu/sample/recorder.py]
(https://github.com/hbldh/pyberryimu/blob/master/pyberryimu/sample/recorder.py)

> Note that the maximum frequency is about 100 Hz for reading from all three IMU sensors 
> (accelerometer, gyroscope and magnetometer). When adding pressure readings it drops to 
> about 10 Hz due to the fact that one has to wait during pressure and temperature reading.
> Run max frequency test script [pyberryimu/sample/max_freq_test.py]
> (https://github.com/hbldh/pyberryimu/blob/master/pyberryimu/sample/max_freq_test.py) to
> get a more complete picture of maximal frequencies.

### Calibration

Calibration of accelerometer is performed using the method described in 
[Frosio, I.; Pedersini, F.; Alberto Borghese, N., 
"Autocalibration of MEMS Accelerometers," 
Instrumentation and Measurement, IEEE Transactions on , 
vol.58, no.6, pp.2034,2041, June 2009]
(http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4655611&isnumber=4919430).

First it prompts the user to position the BerryIMU such that Earth's gravity acts on 
only one of the axes at a time, in both directions. This six point calibration gives a
zero G value and a sensitivity for each axis. At least three more points are needed to
complete the calibration, which can be chose arbitrarily with the only restriction that
the BerryIMU is static. When these have been collected, an optimisation is done to fit
final calibration parameters.

```python
from pyberryimu.client import BerryIMUClient
from pyberryimu.calibration.standard import StandardCalibration

sc = StandardCalibration(verbose=True)
c = BerryIMUClient(bus=1)
sc.calibrate_accelerometer(c)
c.calibration_object = sc
```

When using the `BerryIMUClient` after assigning a `StandardCalibration` to the
`calibration_object` attribute, the readings returned are in the unit `g`.
This calibration can be saved to disc and then loaded later on:

```python
from pyberryimu.client import BerryIMUClient
from pyberryimu.calibration.standard import StandardCalibration

sc = StandardCalibration.load()
with BerryIMUClient(bus=1) as c:
    c.calibration_object = sc
    c.read_accelerometer()
```

Calibration of gyroscope and magnetometer are not implemented yet.

## Documentation

TBD

### BerryIMU links
* [Buy BerryIMU](http://ozzmaker.com/product/berryimu-accelerometer-gyroscope-magnetometer-barometricaltitude-sensor/)
* [BerryIMU Quick Start Guide](http://ozzmaker.com/berryimu-quick-start-guide/)
* [Enable i2c on Raspberry Pi](http://ozzmaker.com/i2c/)
* [BerryIMU Github Repo](http://github.com/mwilliams03/BerryIMU.git)

