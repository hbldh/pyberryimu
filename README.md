# PyBerryIMU

I2C communications lib for using [BerryIMU]
(http://ozzmaker.com/product/berryimu-accelerometer-gyroscope-magnetometer-barometricaltitude-sensor/).

**This library uses and includes several snippets of code form the 
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

To talk to the BerryIMU, a client should be created. It can be created with the `with` command or instantiated 
and opened in a regular fashion.

    from pyberryimu.client import BerryIMUClient
    with BerryIMUClient(bus=1) as bimuc:
        acc = bimuc.read_accelerometers()
        gyro = bimuc.read_gyroscopes()
        mag = bimuc.read_magnetometer()
        pr = bimuc.read_pressure()
        temp = bimuc.read_temperature()
     print acc, gyro, mag, pr, temp
    
## Documentation

Build Sphinx documentation with command

   make html

run from the 

### BerryIMU links
* [Buy BerryIMU](http://ozzmaker.com/product/berryimu-accelerometer-gyroscope-magnetometer-barometricaltitude-sensor/)
* [BerryIMU Quick Start Guide](http://ozzmaker.com/berryimu-quick-start-guide/)
* [Enable i2c on Raspberry Pi](http://ozzmaker.com/i2c/)
* [BerryIMU Github Repo](http://github.com/mwilliams03/BerryIMU.git)

