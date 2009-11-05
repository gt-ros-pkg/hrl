== hrl_camera Theory of Operation ==

This hrl_camera package was designed simpler usage and management of cameras.
It includes 3 components.  First, it includes scripts for making cameras
accessible over ROS.  Second, it provides a method to uniquely reference
different cameras. Finally, it provides a python interfrace for
cameras to be accessed using Python code similar to the example below:

import hrl_camera.hrl_camera as hc
camera_object = hc.find_camera('CAMERA_NAME')

In this case, the camera_object returned to users of this library would be of
the 'correct' or appropriate class for the given 'CAMERA_NAME'.  For example,
if the named camera is a Point Grey Dragonfly2 the object return would be
appropriate for use with Dragonfly's.

To make this little bit of magic happen for a specific camera, two bits of
information are required.  First, the camera's parameters must be in the
file src/hrl_camera/camera_config.py.  As far as this library is concerned,
the particular entry provided in camera_config.py must provide a 'uuid' and
'class' field.  All other fields are specific to the particular camera class
used.  The 'uid' field gives the unique ID of the specific camera (valid for
firewire devices).  The 'class' field provide the name of the python file and
the name of the class found in that file that hrl_camera should instantiate.  
It is important that this file be somewhere in the current process's python
path when hrl_camera runs.

As for the camera class defined, its __init__() method is required to 
take in the configuration dictionary defined in camera_config.py and
have that parameter be the only parameter that must be provided.

=== Utilities ===
./camera_uuid.py
    prints out connected cameras

./hrl_ros_camera.py CAMERA_NAME
    publishes named camera over ROS
    needs camera class to define get_frame()

./ros_camera.py OPENCV_ID
    works with any opencv accessible camera
    publishes its images over ROS

