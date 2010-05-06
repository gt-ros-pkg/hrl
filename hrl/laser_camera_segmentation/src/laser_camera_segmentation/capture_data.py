#! /usr/bin/env python
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
        
import scanner  
import Processor
import configuration    
import hrl_lib.util as ut  
        
# Import Psyco if available
#try:
#    import psyco
#    psyco.full()
#    print "Psyco loaded"
#except ImportError:
#    pass

#from labeling import label_object, scan_dataset, scans_database
#database = scans_database.scans_database()
#database.load('/home/martin/robot1_data/usr/martin/laser_camera_segmentation/labeling','database.pkl')
#dataset = scan_dataset.scan_dataset()
#print dataset

cfg = configuration.configuration('/home/martin/robot1_data/usr/martin/laser_camera_segmentation/labeling')
sc = scanner.scanner(cfg)
pc = Processor.Processor(cfg)

name = ut.formatted_time()
name='2009Oct17_122644'
#sc.capture_and_save(name)
pc.load_data(name)

#pc.load_raw_data('pill_table/2009Sep12_142422')
pc.process_raw_data()
pc.save_mapped_image(name)
pc.display_all_data()
print 'done'



    
    
