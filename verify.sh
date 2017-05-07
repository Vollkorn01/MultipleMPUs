#!/bin/bash

PRJ_PATH="$PWD"

mkdir "$PRJ_PATH/build"
arduino --pref build.path="$PRJ_PATH/build" --verify "$PRJ_PATH/MotionSuit.ino"
arduino --verbose --pref build.path="$PRJ_PATH/build" --board adafruit:avr:feather32u4 --verify "$PRJ_PATH/MotionSuit.ino"
