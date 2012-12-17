#!/bin/bash
#Provide setup for wouse
echo 'SUBSYSTEMS=="usb", ENV{ID_VENDOR_ID}=="214e", ENV{ID_MODEL_ID}=="0001", ENV{ID_INPUT_MOUSE}=="1", NAME="wouse", MODE="0666", OPTIONS+="last_rule"' > /etc/udev/rules.d/wouse.rules
