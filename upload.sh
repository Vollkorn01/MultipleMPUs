#!/bin/bash

PRJ_PATH="$PWD"

mkdir "$PRJ_PATH/build"
arduino --verbose --pref build.path="$PRJ_PATH/build" --board adafruit:avr:feather32u4 --port /dev/ttyACM0 --upload "$PRJ_PATH/MotionSuit.ino"
