#--------------------------------------------------------------
#--    MotioSuit
#--    IMU based motion capture suit
#--------------------------------------------------------------
#--    BQ
#--------------------------------------------------------------
#--    Created by
#--        Alvaro Ferran (alvaroferran)
#--------------------------------------------------------------
#--    Released on January 2016
#--    under the GPL v2
#--------------------------------------------------------------

import bge
import math
from math import *
import mathutils
import time, datetime

import sys
sys.path.append("J:\Programs\Anaconda3\Lib\site-packages")

import serial
import glob

port="COM6"
#port=''.join(glob.glob("/dev/ttyUSB*"))

ser = serial.Serial(port, 115200, timeout=0.5)
print("connected to: " + ser.portstr)
ser.write("a".encode('UTF-8'))


# Get the whole bge scene
scene = bge.logic.getCurrentScene()
# Helper vars for convenience
source = scene.objects
# Get the whole Armature
main_arm = source.get('Armature')
ob = bge.logic.getCurrentController().owner


# IMU serial index to channel mapping
imus = [
    {
        'channel':  'armR',
        'index':    0,
    },
    {
        'channel':  'forearmR',
        'index':    1,
    },
    {
        'channel':  'armL',
        'index':    2,
    },
    {
        'channel':  'forearmL',
        'index':    3,
    },
    {
        'channel':  'trunk',
        'index':    4,
        'corr':     mathutils.Quaternion((1.0, 0.0, 0.0), math.radians(90.0)),
    },
]

# calibration control
start = datetime.datetime.now()
calibrate = True


def updateAngles():
    global start
    global calibrate
    global imus

    # read quaternion update from serial port
    s=ser.readline()[:-3].decode('UTF-8') # remove trailing ";\r\n"
    angles=[x.split(',') for x in s.split(';')]
    for i in range(len(angles)):
        angles[i] = [float(x) for x in angles[i]]

    start_dt = (datetime.datetime.now() - start).total_seconds()

    for imu in imus:
        # create quaternion from serial output
        a = angles[imu['index']]
        q = mathutils.Quaternion((a[0], a[1], a[2], a[3]))

        # handle calibration
        if calibrate:
            if start_dt > 10:
                if 'cal' not in imu:
                    # use current quaternion as calibration zero value
                    imu['cal'] = q.copy().invert()
                else:
                    # apply stored calibration offset
                    q = imu['cal'] * q

        # apply correction if applicable
        if 'corr' in imu:
            q = imu['corr'] * q

        # set quaternion
        ob.channels[imu['channel']].rotation_quaternion = q

    ob.update()
    time.sleep(0.001)
