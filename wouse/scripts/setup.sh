#!/bin/bash
#Provide setup for wouse
echo 'SYSFS{idVendor}=="214e", SYSFS{idProduct}=="0001", NAME="wouse", MODE="0666"' > /etc/udev/rules.d/wouse.rules
