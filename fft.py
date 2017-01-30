#!/usr/bin/env python
#coding=utf8
#========================================
# FFT and Velocity visualisation script 
# Created by Max Dukov
# maxdukov@gmail.com
#========================================
# v. 1.0 RC =)
print "FFT vis script v1.0 RC"
import numpy as np
from numpy import array, arange, abs as np_abs
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from datetime import datetime
import sqlite3

# init sqlite connection
con = sqlite3.connect('/var/www/html/gyro.db')
con.row_factory = lambda cursor, row: row[0]
cur = con.cursor()
"""  use it for create tables"""
#cur.execute('CREATE TABLE IF NOT EXISTS log ( dt VARCHAR(30), x REAL, y REAL, z REAL, orientation INT)')
cur.execute('SELECT x FROM (SELECT * FROM log ORDER BY dt DESC LIMIT 2400) ORDER BY dt ASC;')
x_ = cur.fetchall()
cur.execute('SELECT y FROM (SELECT * FROM log ORDER BY dt DESC LIMIT 2400) ORDER BY dt ASC;')
y_ = cur.fetchall()
cur.execute('SELECT z FROM (SELECT * FROM log ORDER BY dt DESC LIMIT 2400) ORDER BY dt ASC;')
z_ = cur.fetchall()
con.close()
######## process  X 
# freq graph
Yx = np_abs(np.fft.rfft(x_))
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

### plot freq for X
plt.subplot(6, 1, 1)
plt.grid(True)
plt.plot(X,Yx)
plt.xlabel('Freq (Hz)')
# Velocity graph
Vx = [0]*400
for i in range(0,400):
	Vx[i] = x_[i]*(39.3701/(6.28*freq_x))
end = len(Vx)
X = np.asarray(np.linspace(0, 0.5, end, endpoint=True))
### plot Velocity for X
plt.subplot(6, 1, 2)
plt.grid(True)
plt.plot(X,Vx)
plt.xlabel('time (s)')

######## process  Y
# freq graph
Yy = np_abs(np.fft.rfft(y_))
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
plt.subplot(6, 1, 3)
plt.grid(True)
plt.plot(Xy,Yy)
plt.xlabel('Freq (Hz)')
# Velocity graph
Vy = [0]*400
for i in range(0,400):
        Vy[i] = y_[i]*(39.3701/(6.28*freq_y))
end = len(Vy)
Xv = np.asarray(np.linspace(0, 0.5, end, endpoint=True))
### plot Velocity for Y
plt.subplot(6, 1, 4)
plt.grid(True)
plt.plot(Xv,Vy)
plt.xlabel('time (s)')


######## process  Z
# freq graph
Yz = np_abs(np.fft.rfft(z_))
Yz[0] = 0
end = len(Yz)
Xz = np.asarray(np.linspace(0, 400, end, endpoint=True))
id = np.where( Yz == max(Yz))
print "max Yz=", max(Yz)
print "id=", id[0]
freq_z = Xz[id[0]] #
print "freq=", freq_z
### plot freq for Z
plt.subplot(6, 1, 5)
plt.grid(True)
plt.plot(Xz,Yz)
plt.xlabel('Freq (Hz)')
# Velocity graph
Vz = [0]*400
for i in range(0,400):
        Vz[i] = z_[i]*(39.3701/(6.28*freq_z))
end = len(Vz)
Xv = np.asarray(np.linspace(0, 0.5, end, endpoint=True))
### plot Velocity for Z
plt.subplot(6, 1, 6)
plt.grid(True)
plt.plot(Xv,Vz)
plt.xlabel('time (s)')

timestr = datetime.strftime(datetime.now(), '%Y-%m-%d_%H:%M:%S')
plt.savefig('/var/www/html/fft_all_'+timestr+'.png')
