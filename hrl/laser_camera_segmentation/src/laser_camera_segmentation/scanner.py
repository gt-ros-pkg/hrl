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
import hrl_tilting_hokuyo.tilt_hokuyo_servo as ths
#import hokuyo.hokuyo_processing as hp
import hrl_hokuyo.hokuyo_scan as hs


import sys

from labeling import label_object, scan_dataset, scans_database

from opencv import highgui
import hrl_lib.util as ut

class scanner:
    def __init__(self, configuration):
        
        self.config = configuration
        
        
        self.webcam = False
        self.thok = False
        
        self.img = False
        
    
    #use the actual hardware 
    def init_webcam(self):
        if self.webcam:
            return
        
        if self.config.device == 'codyRobot':
            import camera
            self.webcam = camera.Camera(self.config.cam_name)
        else:
            import webcamera
            self.webcam = webcamera.Webcamera(self.config.cam_name, self.config.webcam_id)
	   
        

#        img = False
#        cam_index = -1
#        while not(img) and cam_index < 20:
#	    cam_index = cam_index + 1
#            try:
#		del self.webcam
#		del webcamera
#		import webcamera
 #               self.webcam = webcamera.Webcamera('DesktopWebcam', cam_index)
  #              img = self.webcam.get_frame()
  #          except:
   #             print "Unexpected error:", sys.exc_info()[0]
    #            print "try again...with next webcam" + str(cam_index)
     #           pass
	    
#        if not(img):
 #           print 'ERROR: Webcam init FAILED'
  #          return
   

 
    #use the actual hardware 
    def init_thok(self):
        if self.thok:
            return
        print "Init THOK"
        self.hok = hs.Hokuyo('utm',self.config.thok_hoknum,flip=False)
        self.thok = ths.tilt_hokuyo(self.config.thok_devname,self.config.thok_servonum,self.hok,l1=self.config.thok_l1,l2=self.config.thok_l2) 
        print "Init THOK done"
        
    def capture_image(self):
        self.init_webcam()
        del self.img
        self.img = False
        count = 0
        print 'capture image...'
        
        while not(self.img) and count < 20:
            count = count + 1
            try:
                #call get_frame several times to really get a new picture(!)
                for i in xrange(10):
                    self.img = self.webcam.get_frame()
            except:
                print "Unexpected error:", sys.exc_info()[0]
                print "try again..."
                pass
        print 'count:'+str(count)
        return self.img
    
    def capture_laserscan(self, number_of_scans = 1, angle = None):
        self.init_thok()
        self.laserscans = []
        
        if angle != None:
            tilt_angles = (self.config.thok_tilt_angles[0] + angle, self.config.thok_tilt_angles[1] + angle)
        else:
            tilt_angles = self.config.thok_tilt_angles
        
        for i in range(number_of_scans):
            pos_list,scan_list = self.thok.scan(tilt_angles,speed=self.config.thok_scan_speed,save_scan=False)
            self.laserscans.append((pos_list,scan_list))
            #print scan_list[0]
    
            
        return self.laserscans
    
    
    def save_data(self,name, metadata=True, angle = None):
        dict = {'laserscans' : self.laserscans,
            'l1': self.config.thok_l1, 'l2': self.config.thok_l2,
            'image_angle' : angle} 
        
        prefix = self.config.path+'/data/'+name
        print "Saving: "+prefix+'_laserscans.pkl'
        ut.save_pickle(dict,prefix+'_laserscans.pkl')
        print "Saving: "+prefix+'_image.png'
        highgui.cvSaveImage(prefix+'_image.png',self.img)
        
        if metadata:
            # save metadata to database:
            database = scans_database.scans_database()
            database.load(self.config.path,'database.pkl')
            dataset = scan_dataset.scan_dataset()
            dataset.id = name
            dataset.scan_filename = 'data/'+name+'_laserscans.pkl'
            dataset.image_filename = 'data/'+name+'_image.png'
            database.add_dataset(dataset)
            database.save()
        
        return name
        
    def take_artag_image(self):    
        img = self.capture_image()
        return img
    
    def save_artag_image(self,name):    
        
        filename = self.config.path+'/data/'+name+'_artag_image.png'
        print "Saving: "+filename
        highgui.cvSaveImage(filename,self.img)
        
        return '/data/'+name+'_artag_image.png'
        
    #capture image and laserscans, save image, scan
    def capture_and_save(self,name, metadata=True, angle = None):
        self.init_webcam()
        self.init_thok()
        if None != angle:
            self.thok.servo.move_angle(angle)
            angle = self.thok.servo.read_angle()        
        
        self.capture_image()
        self.capture_laserscan(1, angle)
        return self.save_data(name, metadata, angle)