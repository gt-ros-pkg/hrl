#!/usr/bin/python
#
# Copyright (c) 2010, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

#  \author Martin Schuster (Healthcare Robotics Lab, Georgia Tech.)
import roslib; roslib.load_manifest('laser_camera_segmentation')

print 'TEST CROSSVALIDATION script!!!'

# Import Psyco if available
try:
    import psyco
    psyco.full()
    print "Psyco loaded"
except ImportError:
    pass

import time 
def getTime():
    return '['+time.strftime("%H:%M:%S", time.localtime())+']'
        
import subprocess        
   
print getTime(), 'start'    

import laser_camera_segmentation.processor as processor
import laser_camera_segmentation.configuration as configuration       
        
cfg = configuration.configuration('/home/martin/robot1_data/usr/martin/laser_camera_segmentation/labeling')
pc = processor.processor(cfg)

#pc.calculate_and_save_ground_and_table_transformations_for_all_scans(use_RANSAC_table_plane = False)

pc.generate_save_features(True, True)  


folds = 40       
for current_fold in range(folds): #test_crossvalidation_fold.py
    #output = subprocess.Popen(["python", 'test_crossvalidation_fold.py',str(current_fold)], stdout=subprocess.PIPE, env={"ROS_PACKAGE_PATH":"/home/martin/ros/pkgs:/home/martin/gt-ros-pkg/hrl:/home/martin/robot1/src:/home/martin/gt-ros-pkg/hrl:/home/martin/robot1/src:/home/martin/gt-ros-pkg/hrl:/home/martin/robot1/src", "ROS_ROOT":"/home/martin/ros/ros", "PYTHONPATH": "/home/martin/robot1/src/libraries:/home/martin/robot1/src/libraries/katana:/home/martin/robot1/src/libraries:/home/martin/robot1/src/libraries/katana:/home/martin/robot1/src/libraries:/home/martin/robot1/src/libraries/katana:/home/martin/ros/ros/core/roslib/src:/home/martin/robot1/src/libraries:/home/martin/robot1/src/libraries/katana:/usr/local/lib/python2.5/site-packages/m3rt/:/home/martin/gt-ros-pkg/hrl/segway_omni/src:/home/martin/gt-ros-pkg/hrl/hrl_opencv/src:/home/martin/gt-ros-pkg/hrl/force_torque/src:/home/martin/gt-ros-pkg/hrl/hrl_lib/src:/home/martin/gt-ros-pkg/hrl/zenither/src:/home/martin/gt-ros-pkg/hrl/robotis/src:/usr/local/lib/python2.5/site-packages/m3rt/:/home/martin/gt-ros-pkg/hrl/segway_omni/src:/home/martin/gt-ros-pkg/hrl/hrl_opencv/src:/home/martin/gt-ros-pkg/hrl/force_torque/src:/home/martin/gt-ros-pkg/hrl/hrl_lib/src:/home/martin/gt-ros-pkg/hrl/zenither/src:/home/martin/gt-ros-pkg/hrl/robotis/src:/usr/local/lib/python2.5/site-packages/m3rt/:/home/martin/gt-ros-pkg/hrl/segway_omni/src:/home/martin/gt-ros-pkg/hrl/hrl_opencv/src:/home/martin/gt-ros-pkg/hrl/force_torque/src:/home/martin/gt-ros-pkg/hrl/hrl_lib/src:/home/martin/gt-ros-pkg/hrl/zenither/src:/home/martin/gt-ros-pkg/hrl/robotis/src"}).communicate()[0]
    output = subprocess.Popen(["python", 'test_crossvalidation_fold.py',str(current_fold)]).communicate()[0]
    #print getTime(), output
    
    #todo: parallel: don't use communicate() but just create 3 processes with Popen and then wait for them with wait()



pc.collect_and_save_testresults_crossvalidation(folds)




print getTime(), 'done'