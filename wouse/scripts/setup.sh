#!/bin/bash
#Provide setup for wouse
echo 'SUBSYSTEMS=="usb", KERNEL=="event3", SYSFS{idVendor}=="214e", SYSFS{idProduct}=="0001", NAME="wouse", MODE="0666", OPTIONS+="last_rule"' > /etc/udev/rules.d/wouse.rules
