#!/usr/bin/env python
#coding=utf8
#========================================
# FFT and Velocity visualisation script 
# Created by Max Dukov
# maxdukov@gmail.com
#========================================
# v. 1.0.090417 =)
print "FFT vis script v1.0.090417"
import numpy as np
from numpy import array, arange, abs as np_abs
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from datetime import datetime
import sqlite3
import argparse
import socket
import fcntl
import struct

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])

#==============================
# setup arguments parsing here
#==============================
parser = argparse.ArgumentParser()
parser.add_argument("--sensor", type=int, help="sensor id")
#parser.add_argument("--norm", type=int, help="select normalization mode. 1 for normalization, 0 for standart(default)")
args = parser.parse_args()
if args.sensor is None:
    print "Default sensor id 0x1d will be used. Use ./fft.py --sensor 2 for select second sensor. Use ./fft.py -h for help."
    args.sensor = 1
#if (args.norm is None or args.norm == 0):
#        print "Default mode, no normalization"
#        norm = 0
#if args.norm == 1:
#        print "Normalization mode"
#        norm = 1
#=========================
# init sqlite connection
#=========================
con = sqlite3.connect('/var/www/html/gyro.db')
con.row_factory = lambda cursor, row: row[0]
cur = con.cursor()
cur.execute('SELECT x FROM log WHERE id=? ORDER BY dt ASC;', (str(args.sensor)))
x_ = cur.fetchall()
cur.execute('SELECT y FROM log WHERE id=? ORDER BY dt ASC;', (str(args.sensor)))
y_ = cur.fetchall()
cur.execute('SELECT z FROM log WHERE id=? ORDER BY dt ASC;', (str(args.sensor)))
z_ = cur.fetchall()
con.close()
print "==================================="
print "=> Processing data for sensor id#"+str(args.sensor)
print "==================================="
######## process  X 
# freq graph
Yx = np_abs(np.fft.rfft(x_))
Yx_h = np_abs(np.fft.rfft(x_*np.hanning(len(x_))))
Yx[0] = 0
end = len(Yx)
X = np.asarray(np.linspace(0, 400, end, endpoint=True)) 
id = np.where( Yx == max(Yx))
print "max Yx=", max(Yx)
print "id=", id[0]
freq_x =  X[id[0]] # 
print "freq=", freq_x
print "=============="
### set plt size
fig_size = plt.rcParams["figure.figsize"]
fig_size[0] = 16
fig_size[1] = 22
plt.rcParams["figure.figsize"] = fig_size

fig = plt.figure()
fig.suptitle('FFT and velocity graphs for sensor '+str(args.sensor), fontsize=14, fontweight='bold')
### plot freq for X
plt.subplot(3, 2, 1)
plt.grid(True)
plt.plot(X,Yx,'b.-',label='Row FFT')
plt.plot(X,Yx_h,'c.-',label='FFT with Haning')
plt.xlabel('Freq (Hz)')
plt.legend(loc=0)
# Velocity graph
Vx = [0]*400
for i in range(0,400):
	Vx[i] = x_[i]*(39.3701/(6.28*freq_x))
end = len(Vx)
X = np.asarray(np.linspace(0, 0.5, end, endpoint=True))
### plot Velocity for X
plt.subplot(3, 2, 2)
plt.grid(True)
plt.plot(X,Vx)
plt.xlabel('time (s)')

######## process  Y
# freq graph
Yy = np_abs(np.fft.rfft(y_))
Yy_h = np_abs(np.fft.rfft(y_*np.hanning(len(y_))))
Yy[0] = 0
end = len(Yy)
Xy = np.asarray(np.linspace(0, 400, end, endpoint=True))
id = np.where( Yy == max(Yy))
print "max Yy=", max(Yy)
print "id=", id[0]
freq_y = Xy[id[0]] #
print "freq=", freq_y
print "=============="
### plot freq for Y
plt.subplot(3, 2, 3)
plt.grid(True)
plt.plot(Xy,Yy,'b.-',label='Row FFT')
plt.plot(Xy,Yy_h,'c.-',label='FFT with Hanning')
plt.legend(loc=0)
plt.xlabel('Freq (Hz)')
# Velocity graph
Vy = [0]*400
for i in range(0,400):
        Vy[i] = y_[i]*(39.3701/(6.28*freq_y))
end = len(Vy)
Xv = np.asarray(np.linspace(0, 0.5, end, endpoint=True))
### plot Velocity for Y
plt.subplot(3, 2, 4)
plt.grid(True)
plt.plot(Xv,Vy)
plt.xlabel('time (s)')


######## process  Z
# freq graph
Yz = np_abs(np.fft.rfft(z_))
Yz_h = np_abs(np.fft.rfft(z_*np.hanning(len(z_))))
Yz[0] = 0
end = len(Yz)
Xz = np.asarray(np.linspace(0, 400, end, endpoint=True))
id = np.where( Yz == max(Yz))
print "max Yz=", max(Yz)
print "id=", id[0]
freq_z = Xz[id[0]] #
print "freq=", freq_z
### plot freq for Z
plt.subplot(3, 2, 5)
plt.grid(True)
plt.plot(Xz,Yz,'b.-',label='Row FFT')
plt.plot(Xz,Yz_h,'c.-',label='FFT with Hanning')
plt.legend(loc=0)
plt.xlabel('Freq (Hz)')
# Velocity graph
Vz = [0]*400
for i in range(0,400):
        Vz[i] = z_[i]*(39.3701/(6.28*freq_z))
end = len(Vz)
Xv = np.asarray(np.linspace(0, 0.5, end, endpoint=True))
### plot Velocity for Z
plt.subplot(3, 2, 6)
plt.grid(True)
plt.plot(Xv,Vz)
plt.xlabel('time (s)')

timestr = datetime.strftime(datetime.now(), '%Y-%m-%d_%H:%M:%S')
#if norm == 0:
plt.savefig('/var/www/html/fft_all_'+timestr+'_'+(str(args.sensor))+'.png')
#if norm == 1:
#	plt.savefig('/var/www/html/fft_all_norm_'+timestr+'_'+(str(args.sensor))+'.png')
print 'Please have a look on results here: http://'+(str(get_ip_address('eth0')))+'/fft_all_'+timestr+'_'+(str(args.sensor))+'.png'
