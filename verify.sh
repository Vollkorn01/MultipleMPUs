#!/bin/bash

PRJ_PATH="$PWD"

mkdir "$PRJ_PATH/build"
arduino --pref build.path="$PRJ_PATH/build" --verify "$PRJ_PATH/MotionSuit.ino"
