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
import time

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


# calibration and serial index to channel map
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
        'index':    3,
        'corr':     mathutils.Quaternion((1.0, 0.0, 0.0), math.radians(90.0)),
    },
]


def updateAngles():
    # read quaternion update from serial port
    s=ser.readline()[:-3].decode('UTF-8') # remove trailing ";\r\n"
    angles=[x.split(',') for x in s.split(';')]
    for i in range(len(angles)):
        angles[i] = [float(x) for x in angles[i]]

    global imus
    for imu in imus:
        # create quaternion from serial output
        a = angles[imu['index']]
        q = mathutils.Quaternion((a[0], a[1], a[2], a[3]))

        # apply correction if applicable
        if 'corr' in imu:
            q = imu['correction'] * q

        # set quaternion
        ob.channels[imu['channel']].rotation_quaternion = q

    ob.update()
    time.sleep(0.001)
