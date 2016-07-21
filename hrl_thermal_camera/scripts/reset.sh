#!/bin/bash -x

killall -9 tvtime
sleep 1
tvtime -v -g1x1 -i1 -d /dev/video0 &
