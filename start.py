#!/usr/bin/env python
#coding=utf8
#========================================
# MMA8451 data logging script
# Created by Max Dukov
# based on mma8451.py - Python API for MMA8451 accelerometer.  Author: jatin kataria
# maxdukov@gmail.com
#========================================
# v. 1.0.280317 =)

print "MMA8451 data collection script v1.0.280317"


import smbus
import sys
import os
import time
import datetime
import sqlite3
import argparse
import sched

#==============================
# setup arguments parsing here 
#==============================
parser = argparse.ArgumentParser()
parser.add_argument("--sensor", type=int, help="select sensor id, 1 for 0x1d and 2 for 0x1c") 
parser.add_argument("--seq", type=int, help="select sequential mode. 1 for sequential start, 0 for single sensor(default)")
#parser.add_argument("--norm", type=int, help="select normalization mode. 1 for normalization, 0 for standart(default)")
#parser.add_argument("--ncount", type=int, help="select normalization steps count. Default - 5 steps")
#parser.add_argument("--ntime", type=int, help="select normalization time between data collection in minutes. Default - 1m.")
args = parser.parse_args()
if args.sensor is None and (args.seq is None or args.seq == 0):
	print "Default sensor id 0x1d will be used. Use ./start.py --sensor 2 for select second sensor. Use ./start.py -h for help."
	DEFAULT_ADDRESS = 0x1d
	args.sensor = 1
#if (args.norm is None or args.norm == 0):
#	print "Default mode, no normalization"
#	norm = 0
#if args.norm == 1:
#        print "Normalization mode"
#        norm = 1

if (args.seq is None or args.seq == 0):
	print "Default mode, single sensor run"
        seq = 0
if args.seq == 1:
        print "Sequential mode"
        seq = 1

#if (args.ncount is None and norm == 1):
#        print "Default mode normalization count (5)."
#        norm_count = 5
#if (args.ncount and norm == 1):
#        print "Normalization count="+str(args.ncount)
#        norm_count = args.ncount 
#if (args.ntime is None and norm == 1):
#        print "Default normalization time (1 min)."
#        norm_time = 1
#if (args.ntime and norm == 1):
#        print "Normalization time="+str(args.ntime)
#        norm_time = args.ntime

#============================================
# Scheduler init
#============================================
#scheduler = sched.scheduler(time.time, time.sleep)

#==============================
# SQLite setup
#==============================
# init sqlite connection
con = sqlite3.connect('/var/www/html/gyro.db')
cur = con.cursor()

#============================================
# check and create tables, if it's not exists
#============================================
# cur.execute('DROP TABLE log') 
# cur.execute('DROP TABLE archive')
cur.execute('CREATE TABLE IF NOT EXISTS log     ( dt VARCHAR(30), id INT, x INT, y INT, z INT, orientation INT, norm BOOLEAN default 0)')
cur.execute('CREATE TABLE IF NOT EXISTS archive ( dt VARCHAR(30), id INT, x INT, y INT, z INT, orientation INT, norm BOOLEAN default 0, uploaded BOOLEAN default 0)')
#cur.execute('CREATE TABLE IF NOT EXISTS norm    ( count INT, id INT, x INT, y INT, z INT, orientation INT)')
cur.execute('CREATE INDEX IF NOT EXISTS upld ON archive(uploaded)')
cur.execute('DELETE FROM log')
con.commit()

#==============================
# MMA8451 setup
#==============================
HIGH_RES_PRECISION = 14
LOW_RES_PRECISION = 8
DEACTIVATE = 0
ACTIVATE = 1
EARTH_GRAVITY_MS2 = 9.80665

BW_RATE_800HZ = 0x0
BW_RATE_400HZ = 0x1
BW_RATE_200HZ = 0x2
BW_RATE_100HZ = 0x3
BW_RATE_50HZ = 0x4
BW_RATE_12_5HZ = 0x5
BW_RATE_6_25HZ = 0x6
BW_RATE_1_56HZ = 0x7   # frequency for Low power

RANGE_2G = 0x00
RANGE_4G = 0x01
RANGE_8G = 0x02

#DEFAULT_ADDRESS = 0x1d # may be 0x1c for "China version" or for moddded Adafruit
if args.sensor == 1:
    DEFAULT_ADDRESS = 0x1d
    print "sensor 0x1d selected, default for Adafuit"
if args.sensor == 2:
    DEFAULT_ADDRESS = 0x1c
    print "sensor 0x1c selected."

MMA8451_REG_OUT_X_MSB = 0x01
MA8451_REG_SYSMOD = 0x0B
MA8451_REG_WHOAMI = 0x0D
MMA8451_REG_XYZ_DATA_CFG = 0x0E
MMA8451_REG_F_SETUP = 0x9
MMA8451_REG_PL_STATUS = 0x10
MMA8451_REG_PL_CFG = 0x11
MMA8451_REG_CTRL_REG1 = 0x2A
MMA8451_REG_CTRL_REG2 = 0x2B
MMA8451_REG_CTRL_REG4 = 0x2D
MMA8451_REG_CTRL_REG5 = 0x2E
MMA8451_RESET = 0x40

ORIENTATION_ON = 0x40
MAX_BLOCK_LEN = 32

I2C_SLAVE = 0x0703
I2C_SMBUS = 0x0720
I2C_SMBUS_WRITE = 0
I2C_SMBUS_READ = 1
I2C_SMBUS_BYTE_DATA = 2
I2C_SMBUS_BLOCK_DATA = 5
I2C_SMBUS_I2C_BLOCK_DATA = 8
I2C_SMBUS_PROC_CALL = 4

DATA_RATE_BIT_MASK = 0xc7
MODS_LOW_POWER = 3

REDUCED_NOIDE_MODE = 0
OVERSAMPLING_MODE = 1

HIGH_RES_MODE = {
    REDUCED_NOIDE_MODE: [MMA8451_REG_CTRL_REG1, 0x4],
    OVERSAMPLING_MODE: [MMA8451_REG_CTRL_REG2, 0x2],
}

RANGE_DIVIDER = {
    RANGE_2G: 4096 / EARTH_GRAVITY_MS2,
    RANGE_4G: 2048 / EARTH_GRAVITY_MS2,
    RANGE_8G: 1024 / EARTH_GRAVITY_MS2,
}

ORIENTATION_POS = {
    0: 'Portrait Up Front',
    1: 'Portrait Up Back',
    2: 'Portrait Down Front',
    3: 'Portrait Down Back',
    4: 'Landscape Right Front',
    5: 'Landscape Right Back',
    6: 'Landscape Left Front',
    7: 'Landscape Left Back',
}
BCM2708_COMBINED_PARAM_PATH = '/sys/module/i2c_bcm2708/parameters/combined'
BOARD_CPUINFO = '/proc/cpuinfo'


class MMA8451(object):

    def __init__(self, address, bus=None,
                 sensor_range=RANGE_4G, data_rate=BW_RATE_800HZ, debug=False):
        self.i2c_address = address
        if bus is None:
            bus = self.get_pi_revision()
        # bcm2708 doesnt support repeated start bits for i2c messages.
        # set the combined parameter which sends command correctly.
        # **********WARNING*******************
        # axes data seems to have lot of noise and its a hack.

        os.chmod(BCM2708_COMBINED_PARAM_PATH, 666)
        os.system('echo -n 1 > {!s}'.format(BCM2708_COMBINED_PARAM_PATH))

        # ************************************

        self._smbus = smbus.SMBus(bus)
        self.sensor_range = sensor_range
        self.data_rate = data_rate
        self.high_res_mode = OVERSAMPLING_MODE
        self.setup_sensor()

    def __del__(self):
        """ destructor for smbus open file """
        """logger.debug('closing smbus file descriptor.')"""
        self._smbus.close()

    def set_high_res_mode(self, mode=OVERSAMPLING_MODE):
        """ set the type of high res mode."""
        self.high_res_mode = mode

    def get_pi_revision(self):
        """ get pi revision. """
        # select the correct i2c bus for this revision of Raspberry Pi
        revision = ([l[12:-1] for l in open(BOARD_CPUINFO, 'r').readlines()
                     if l[:8] == "Revision"] + ['0000'])[0]
        bus_num = 1 if int(revision, 16) >= 4 else 0
        return bus_num

    def set_data_rate(self, data_rate):
        """ set the data rate for the sensor. """
        self.data_rate = data_rate
        current = self.read_byte_data(MMA8451_REG_CTRL_REG1)
        # deactivate
        self.write_byte_data(MMA8451_REG_CTRL_REG1, DEACTIVATE)
        current &= DATA_RATE_BIT_MASK
        current |= (data_rate << 3)
        self.write_byte_data(MMA8451_REG_CTRL_REG1, current | ACTIVATE)

    def read_byte_data(self, register):
        """ read data from a specific register of the sensor.
        Arguments:
            register: Register value from which data has to be read.
        Returns:
            Data read.
        """
	time.sleep(0.0001)
        return self._smbus.read_byte_data(self.i2c_address, register)

    def write_byte_data(self, register, value):
        """ write byte data to a destination register.
        Arguments:
            register: Register value where value has to be written.
            value: Value to be written. Max value is 2**8 - 1.
        """
        self._smbus.write_byte_data(self.i2c_address, register, value)

    def read_block_data(self, register, l):
        """ read a block of data of given len from smbus.
        Arguments:
            register: Register value from which data has to be read.
            l: length of data to be read. Value is capped at 32 bytes.
        Returns:
            Data read in a list.
        """
        # *********** WARNING ****************
        # crashes the kernel
        time.sleep(0.0001)
        le = l if l <= MAX_BLOCK_LEN else MAX_BLOCK_LEN
        return self._smbus.read_block_data(self.i2c_address, register, le)

    def read_i2c_block_data(self, register, l):
        """ read i2c block of data of given length from i2c bus.
        Arguments:
            register: Register value from which data has to be read.
            l: length of data to be read. Value is capped at 32 bytes.
        Returns:
            Data read in a list.
        """
  	"""      time.sleep(0.3)"""
        le = l if l <= MAX_BLOCK_LEN else MAX_BLOCK_LEN
        return self._smbus.read_i2c_block_data(self.i2c_address, register, le)

    def set_resolution(self, resolution='high'):
        """sets the resolution to high(14-bit) or low(8-bit).
        Arguments:
            resolution: (high | low)
        """
        if resolution == 'high':
            self.write_byte_data(HIGH_RES_MODE[self.high_res_mode][0],
                                 HIGH_RES_MODE[self.high_res_mode][1])
        else:
            for reg, val in HIGH_RES_MODE.values():
                self.write_byte_data(reg, 0)

    def set_low_power(self):
        """ Enable low power mode for sensor. """
        self.write_byte_data(MMA8451_REG_CTRL_REG2, MODS_LOW_POWER)

    def setup_sensor(self):
        """ Initial setup the sensor for measurement """
        device_id = self.read_byte_data(MA8451_REG_WHOAMI)
	#        if device_id != 0x1A:
	#      	     raise Exception("No MMA8451 detected. Detected Device ID: %s" %
	#                            (str(device_id)))
        self.reset_sensor()
        self.set_resolution()

        # data ready int1
        self.write_byte_data(MMA8451_REG_CTRL_REG4, 0x1)
        self.write_byte_data(MMA8451_REG_CTRL_REG5, 0x1)

        # turn on orientation
        self.write_byte_data(MMA8451_REG_PL_CFG, ORIENTATION_ON)
        # activate at max rate, low noise mode
        self.write_byte_data(MMA8451_REG_CTRL_REG1,
                             0x4 | ACTIVATE)

        # set range and data rate
        self.set_range(self.sensor_range)
        self.set_data_rate(self.data_rate)

    def reset_sensor(self):
        """ Resets the sensor. """
        self.write_byte_data(MMA8451_REG_CTRL_REG2, MMA8451_RESET)
        while self.read_byte_data(MMA8451_REG_CTRL_REG2) != 0:
            time.sleep(1)

    def set_range(self, value):
        """ Set the range for the sensor.
        Arguments:
            Value: Representing the mode for the sensor.
        """
        # sensitivity max is 4g in case of reduced noise mode.
        if self.high_res_mode == REDUCED_NOIDE_MODE and value == RANGE_8G:
            value = RANGE_4G
        self.sensor_range = value & 3
        # first standby
        reg1 = self.read_byte_data(MMA8451_REG_CTRL_REG1)
        self.write_byte_data(MMA8451_REG_CTRL_REG1, DEACTIVATE)
        self.write_byte_data(MMA8451_REG_XYZ_DATA_CFG, self.sensor_range)
        self.write_byte_data(MMA8451_REG_CTRL_REG1, reg1 | ACTIVATE)

    def get_range(self):
        """ Get current range mode for the sensor.
        Returns: Range Mode Value.
        """
        return (self.read_byte_data(MMA8451_REG_XYZ_DATA_CFG) & 3)

    def get_orientation(self):
        """ Get current orientation of the sensor.
            Returns: Orientaton number for the sensor.
        """
        orientation = self.read_byte_data(MMA8451_REG_PL_STATUS) & 0x7

        return orientation

    def _validate_axes_readings(self):
        """ make sure F_READ and F_MODE are disabled."""
        f_read = self.read_byte_data(MMA8451_REG_CTRL_REG1) & 2
        assert f_read == 0, 'F_READ mode is not disabled. : %s' % (f_read)
        f_mode = self.read_byte_data(MMA8451_REG_F_SETUP) & 0xC0
        assert f_mode == 0, 'F_MODE mode is not disabled. : %s' % (f_mode)

    def get_axes_measurement(self):
        """ Get current measurements for x, y, z axes of the sensor.
            Returns: {x:X, y:Y, z:Z} a dict representing the values in
            m/s2.
        """
	
        self._validate_axes_readings()
        read_bytes = self.read_i2c_block_data(MMA8451_REG_OUT_X_MSB, 6)
        if self.high_res_mode is not None:
            x = ((read_bytes[0] << 8) | read_bytes[1]) >> 2
            y = ((read_bytes[2] << 8) | read_bytes[3]) >> 2
            z = ((read_bytes[4] << 8) | read_bytes[5]) >> 2
            precision = HIGH_RES_PRECISION
        else:
            x = (read_bytes[0] << 8)
            y = (read_bytes[1] << 8)
            z = (read_bytes[2] << 8)
            precision = LOW_RES_PRECISION
        max_val = 2 ** (precision - 1) - 1
        signed_max = 2 ** precision

        x -= signed_max if x > max_val else 0
        y -= signed_max if y > max_val else 0
        z -= signed_max if z > max_val else 0

        x = round((float(x)) / RANGE_DIVIDER[self.sensor_range], 3)
        y = round((float(y)) / RANGE_DIVIDER[self.sensor_range], 3)
        z = round((float(z)) / RANGE_DIVIDER[self.sensor_range], 3)

        return {"x": x, "y": y, "z": z}
    def get_data_bin(self,size, norm, last, id):
	dt_list = [0]*size
	x_list = [0]*size
	y_list = [0]*size
	z_list = [0]*size
	o_list = [0]*size
	t = 0
	start = datetime.datetime.now()
	run = datetime.datetime.now()
	print '=== Sensor settings:'
	print '==> Range = 4G'
	print '==> Data rate = 800Hz'
	print '==> Resolution = High(14-bit)'
	print '==> Oversampling Mode = On'
	print '============================='
	print '=== data collection started. Start time='+str(run)
	while t < size:
        	if (run + datetime.timedelta(0,0,1230))  <= datetime.datetime.now():
                	run = datetime.datetime.now()
	                axes = mma8451.get_axes_measurement()
	                dt_list[t] = run
        	        x_list[t] =  (axes['x'])
                	y_list[t] =  (axes['y'])
	                z_list[t] =  (axes['z'])
        	        o_list[t] =  (mma8451.get_orientation())
                	t = t+1
	if norm == 0:
	        print '=== data collection finished, start data saving'
        	t = 0
	        while t < size:
        	        cur.execute('INSERT INTO log     (dt, id, x, y, z, orientation ) VALUES(?, ?, ?, ?, ?, ?)',(dt_list[t], args.sensor, x_list[t], y_list[t], z_list[t], o_list[t]))
                	cur.execute('INSERT INTO archive (dt, id, x, y, z, orientation ) VALUES(?, ?, ?, ?, ?, ?)',(dt_list[t], args.sensor, x_list[t], y_list[t], z_list[t], o_list[t]))
                	t = t+1
        	con.commit()
#	if norm == 1 and last == 0:
#        	print '=== norm data collection finished, start data saving'
#        	t = 0
#        	while t < size:
#                	cur.execute('INSERT INTO norm     (count, id, x, y, z, orientation ) VALUES( ?, ?, ?, ?, ?, ?)',( t, args.sensor, x_list[t], y_list[t], z_list[t], o_list[t]))
#                	t = t+1
#        	con.commit()
#	if  norm == 1 and last == 1:
#        	print '=== data normalization '
#         	cur.execute('INSERT INTO log     (dt, id, x, y, z, orientation, norm) SELECT datetime(\'now\'), id, avg(x), avg(y), avg(z), min(orientation), 1 FROM norm GROUP BY count')
#         	cur.execute('DELETE FROM norm')
#         	con.commit()

	return ()

if __name__ == "__main__":
#========================
# run with normalization 
#========================
#    if  norm == 1:
#	     while norm_count > 0:
#             		scheduler.enter(norm_time*norm_count, 1, mma8451.get_data_bin, (2400,1,0))
#        		norm_count = norm_count - 1
#	     scheduler.run()
#	     mma8451.get_data_bin(2400,1,1) # calc normalized value
	
#==========================
# run witout normalisation 
#==========================
    if  seq == 0:
	    mma8451 = MMA8451(DEFAULT_ADDRESS)
	    mma8451.get_data_bin(2400,0,0,args.sensor)
	    con.close
	    print '=== data saving finished, data visualisation starting'
	    cmd = "./fft.py --sensor "+str(args.sensor)
    	    os.system(cmd)

    if  seq == 1:
	    DEFAULT_ADDRESS = 0x1d
	    mma8451 = MMA8451(DEFAULT_ADDRESS)
	    mma8451.get_data_bin(2400,0,0,1)
	    mma8451 = MMA8451(DEFAULT_ADDRESS)
	    DEFAULT_ADDRESS = 0x1c
	    mma8451.get_data_bin(2400,0,0,2)	    
	    con.close
    	    print '=== data saving finished, data visualisation starting for sensor 1'
            cmd = "./fft.py --sensor 1" 
            os.system(cmd)
            print '=== data saving finished, data visualisation starting for sensor 2'
            cmd = "./fft.py --sensor 2"
            os.system(cmd)
