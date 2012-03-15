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
#import processor
import configuration    
#import hrl_lib.util as ut  
#
#import roslib; roslib.load_manifest('laser_camera_segmentation')
#import hrl_hokuyo.hokuyo_scan as hs
#import hrl_hokuyo.hokuyo_scan as hokuyo_scan


#import opencv as cv
#from opencv import highgui
#
#import pylab
#from matplotlib.patches import  Rectangle
#
#my_svm = cv.CvSVM()
##print CvSVM::train(const CvMat* _train_data, const CvMat* _responses, const CvMat* _var_idx=0, const CvMat* _sample_idx=0, CvSVMParams _params=CvSVMParams())
#train_data = cv.cvCreateMat(10,2,cv.CV_32FC1) #CvMat* cvCreateMat(int rows, int cols, int type)
#train_data[0][0] = 1
#train_data[1][0] = 2
#train_data[2][0] = 3
#train_data[3][0] = 4
#train_data[4][0] = 5
#train_data[5][0] = 6
#train_data[6][0] = 7
#train_data[7][0] = 8
#train_data[8][0] = 9
#train_data[9][0] = 10
#train_data[0][1] = 1
#train_data[1][1] = 2
#train_data[2][1] = 3
#train_data[3][1] = 4
#train_data[4][1] = 5
#train_data[5][1] = 6
#train_data[6][1] = 7
#train_data[7][1] = 8
#train_data[8][1] = 9
#train_data[9][1] = 10
#
#for i in range(10):
#    print train_data[i][0]
#    print train_data[i][1]
#    print '###'  
#
#responses = cv.cvCreateMat(10,1,cv.CV_32FC1)
#responses[0] = 1
#responses[1] = 1
#responses[2] = 1
#responses[3] = 1
#responses[4] = 1
#responses[5] = 0
#responses[6] = 0
#responses[7] = 0
#responses[8] = 0
#responses[9] = 0
#
#
#params = cv.CvSVMParams()
#params.svm_type = cv.CvSVM.C_SVC
##      Type of SVM, one of the following types: 
##      CvSVM::C_SVC - n-class classification (n>=2), allows imperfect separation of classes with penalty multiplier C for outliers. 
##      CvSVM::NU_SVC - n-class classification with possible imperfect separation. Parameter nu (in the range 0..1, the larger the value, the smoother the decision boundary) is used instead of C. 
##      CvSVM::ONE_CLASS - one-class SVM. All the training data are from the same class, SVM builds a boundary that separates the class from the rest of the feature space. 
##      CvSVM::EPS_SVR - regression. The distance between feature vectors from the training set and the fitting hyperplane must be less than p. For outliers the penalty multiplier C is used. 
##      CvSVM::NU_SVR - regression; nu is used instead of p. 
#params.kernel_type = cv.CvSVM.SIGMOID 
##CvSVM::LINEAR - no mapping is done, linear discrimination (or regression) is done in the original feature space. It is the fastest option. d(x,y) = x*y == (x,y)
##CvSVM::POLY - polynomial kernel: d(x,y) = (gamma*(x*y)+coef0)degree 
##CvSVM::RBF - radial-basis-function kernel; a good choice in most cases: d(x,y) = exp(-gamma*|x-y|2) 
##CvSVM::SIGMOID - sigmoid function is used as a kernel: d(x,y) = tanh(gamma*(x*y)+coef0) 
#
#print my_svm.train_auto(train_data, responses,None,None,params)
#print my_svm.get_params()
#test = cv.cvCreateMat(1,2,cv.CV_32FC1)
#test[0] = 6
#test[1] = 8.7878
#print my_svm.predict(test)
#
#import matplotlib.pyplot as plt
#import matplotlib.image as mpimg
#import numpy as np
#
#n = 100
#m = 100
#results = np.array(-1*np.ones((n,m)))
#
#for i in range(n):
#    for j in range(m):
#        test[0]=i
#        test[1]=j
#        results[i][j] = my_svm.predict(test)
#        #print str(i) + ' ' + str(j) + ' ' + ' -> ' + str(results[i][j])
#
##print results
#
#imgplot = plt.imshow(results, cmap=pylab.cm.gray, interpolation='nearest')
##imgplot = plt.imshow(np.array(train_data).transpose())
##imgscatter = plt.scatter(np.array(train_data)[:,0], np.array(train_data)[:,1])
#plt.show()
#
#
#
##pylab.ion() #interactive
##pylab.figure(figsize=(8,4))
##pylab.hold(True)
##pylab.subplot(121)
##pylab.title('test')
##pylab.imshow(responses, cmap=pylab.cm.gray, interpolation='nearest')
##
##pylab.draw()
#
#
#
##cfg = configuration.configuration('/home/martin/robot1_data/usr/martin/laser_camera_segmentation/labeling')
###sc = scanner.scanner(cfg)
##pc = processor.processor(cfg)
##
###name = ut.formatted_time()
###sc.capture_and_save(name)
###pc.load_raw_data(name)
##
##id = '2009Sep14_095609'
##pc.load_raw_data(id)
##pc.load_metadata(id)
##print pc.scan_dataset.id
##print pc.scan_dataset.polygons
##pc.create_polygon_images()
##pc.process_raw_data()
###pc.save_mapped_image(name)
##pc.display_all_data()
##
##print pc.scan_dataset.polygons[0].cvImage[400]
#
#
##! /usr/bin/env python
#
#        
#        
cfg = configuration.configuration('/home/martin/robot1_data/usr/martin/laser_camera_segmentation/calib')
cfg.webcam_id = 0
sc = scanner.scanner(cfg)
##pc = processor.processor(cfg)
##
###name = ut.formatted_time()
sc.capture_and_save('calib')
##
##pc.load_raw_data('2009Oct17_114217')
##pc.process_raw_data()
##pc.display_all_data()
#
#

print 'done'



    
    