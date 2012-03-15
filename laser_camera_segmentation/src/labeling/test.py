#!/usr/bin/python
import roslib; roslib.load_manifest('laser_camera_segmentation')

print 'TEST script!!!'

# Import Psyco if available
try:
    import psyco
    psyco.full()
    print "Psyco loaded"
except ImportError:
    pass

import laser_camera_segmentation.processor as processor
import laser_camera_segmentation.configuration as configuration       
        
import time 

def getTime():
    return '['+time.strftime("%H:%M:%S", time.localtime())+']'
        
        
def generate_train_save():        
    #pc.load_data(id)
    #print getTime(), 'generate and save features...'   
    #pc.generate_save_features(True, True)  
    pc.load_data(id)
    print getTime(), 'train_and_save_Classifiers...'   
    pc.train_and_save_Classifiers()

def print_test():
    print getTime(), 'testing:',pc.feature_type,'k=',pc.feature_neighborhood,'r=',pc.feature_radius
   
   
   
print getTime(), 'start'    
        
        
cfg = configuration.configuration('/home/martin/robot1_data/usr/martin/laser_camera_segmentation/labeling')
#sc = scanner.scanner(cfg)
pc = processor.processor(cfg)
#generate all features and train Classifiers
id = '2009Oct30_162400'



#pc.classifier_training_size = 1000000000
#pc.feature_neighborhood = 20
#pc.feature_radius = 0.03
#pc.feature_type = 'gaussian_histograms'
#print_test()
#generate_train_save()
#pc.load_Classifiers()
#pc.test_classifiers_on_testset()
#pc.update_test_postprocessing_on_testset()

labels, testresults = pc.load_classifier_and_test_on_dataset('all_post', id)
print 'testresults', testresults
import numpy as np
print np.shape(labels)
print labels

import sys

#sys.exit()



print getTime(), 'done'




