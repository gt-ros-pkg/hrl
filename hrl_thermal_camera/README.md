# HRL Thermal Camera Driver

This package contains a ROS Driver for FLIR Tau2 Thermal Camera connected to the computer using a
Pleora iPort Cameralink to USB 3.0 using Pleora SDK

Some info on a package this code is based on:
https://github.com/sam17/ace_USB3

## Supported hardware
This driver should work at least with a FLIR Tau2 Thermal Camera connected to the computer using a
Pleora iPort Cameralink to USB3 converter.

## ROS API

### ros_pleora

`hrl_thermal_camera` is a driver for a FLIR Tau2 Thermal Camera.

### Published topics

This section is not complete or accurate

`~image_raw` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
    The unprocessed image data.

## Installing Pleora eBUS SDK
You can find the latest version of Pleora eBus SDK from [here](http://www.pleora.com/support-center/documentation-downloads). eBus SDK 4.0.7 is in the install folder of this driver.

To install, run the following command:

```bash
cd install
sudo sh setup-usb.sh
sudo ./eBUS_SDK_X.X.X.X_<etc>.run
# accept all default options
sudo /opt/pleora/ebus_sdk/<your_linux_distribution>/bin/install_daemon.sh
source /opt/pleora/ebus_sdk/<your_linux_distribution>/bin/set_puregev_env
```
If you are using ubuntu 14.04, you need to install `libudev-dev` and link it into `/usr/lib`, since eBUS SDK links to that version by default.

```bash
sudo apt-get install libudev-dev
cd /usr/lib
sudo ln -s <your_linux_distribution>/libudev.so libudev.so.1
```

Go to /opt/pleora/ebus_sdk/<your_linux_distribution>/share/samples and test using PvDeviceFinder. If it, works, The driver will work.

Adding ```source /opt/pleora/ebus_sdk/<your_linux_distribution>/bin/set_puregev_env ``` to .bashrc is advised.

This will install the eBUS SDK to `/opt/pleora`.

## Running the node
First source /opt/pleora/ebus_sdk/<your_linux_distribution>/bin/set_puregev_env
Then run /opt/pleora/ebus_sdk/<your_linux_distribution>/bin/eBUSPlayer
Select the device from the list and connect.
Under Device Control -> Image Format Control, set in this order:
1) Test Pattern: Off
2) Pixel Format: Mono16
3) Width: 324
4) Height: 256

Order matters (specifically you cannot set the width to 324 if pixel format is not Mono16)

Then run the driver. Just run the launch file and select the camera from the list in the command prompt.
The launch needs screen set so you can get the prompt to select the device.

```
roslaunch hrl_thermal_camera thermal_camera.launch
```
