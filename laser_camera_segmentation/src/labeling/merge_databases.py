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
from labeling import label_object, scan_dataset, scans_database

import copy

##WARNING! THIS DOES A PARTIAL COPY OF A DATABASE! BACKUP BEFOREHAND AND KNOW WHAT YOU'RE DOING!!##

path = '/home/martin/robot1_data/usr/martin/laser_camera_segmentation/labeling'
f1 = 'database_changed.pkl'
db1 = scans_database.scans_database()
db1.load(path, f1)

f2 = 'database.pkl'
db2 = scans_database.scans_database()
db2.load(path, f2)

#assume db2 is equal or larger, copy changes from db1 to db2!
d2 = db2.get_first_dataset()
d1 = db1.get_first_dataset()
while False != d1:

    if False != d2 and d1.id == d2.id:
        print 'copy', d1.id
        d2.dict = copy.deepcopy(d1.dict)
        
    d2 = db2.get_next_dataset()
    d1 = db1.get_next_dataset()



db2.save()
