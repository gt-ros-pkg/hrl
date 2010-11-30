## Copyright (c) 2004-2007, Andrew D. Straw. All rights reserved.

## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are
## met:

##     * Redistributions of source code must retain the above copyright
##       notice, this list of conditions and the following disclaimer.

##     * Redistributions in binary form must reproduce the above
##       copyright notice, this list of conditions and the following
##       disclaimer in the documentation and/or other materials provided
##       with the distribution.

##     * Neither the name of the Andrew D. Straw nor the names of its
##       contributors may be used to endorse or promote products derived
##       from this software without specific prior written permission.

## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
## "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
## LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
## A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
## OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
## SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
## LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
## DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
## THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


##Original code was modified, PlaneLeastSquaresModel() added:
#  \author Martin Schuster (Healthcare Robotics Lab, Georgia Tech.)


import numpy as np
import scipy # use numpy if scipy unavailable
import scipy.linalg # use numpy if scipy unavailable

### Optional imports below
#    import processor    [Both only for testing obsolete dataset, testPlanePointcloud() ]
#    import configuration  
#    import pylab;  import sys


TEST_FOLDER ='/home/jokerman/svn/robot1_data/usr/martin/laser_camera_segmentation/labeling'

def ransac(data,model,n,k,t,d,debug=False,return_all=False):
    print 'INFO: running RANSAC for k=',k,'iterations'
    """fit model parameters to data using the RANSAC algorithm
    
This implementation written from pseudocode found at
http://en.wikipedia.org/w/index.php?title=RANSAC&oldid=116358182

{{{
Given:
    data - a set of observed data points
    model - a model that can be fitted to data points
    n - the minimum number of data values required to fit the model
    k - the maximum number of iterations allowed in the algorithm
    t - a threshold value for determining when a data point fits a model
    d - the number of close data values required to assert that a model fits well to data
Return:
    bestfit - model parameters which best fit the data (or nil if no good model is found)
iterations = 0
bestfit = nil
besterr = something really large
while iterations < k {
    maybeinliers = n randomly selected values from data
    maybemodel = model parameters fitted to maybeinliers
    alsoinliers = empty set
    for every point in data not in maybeinliers {
        if point fits maybemodel with an error smaller than t
             add point to alsoinliers
    }
    if the number of elements in alsoinliers is > d {
        % this implies that we may have found a good model
        % now test how good it is
        bettermodel = model parameters fitted to all points in maybeinliers and alsoinliers
        thiserr = a measure of how well model fits these points
        if thiserr < besterr {
            bestfit = bettermodel
            besterr = thiserr
        }
    }
    increment iterations
}
return bestfit
}}}
"""
#    iterations = 0
#    bestfit = None
#    besterr = np.inf
#    best_inlier_idxs = None
#    while iterations < k:
#        maybe_idxs, test_idxs = random_partition(n,data.shape[0])
#        print n
#        maybeinliers = data[maybe_idxs,:]
#        #print 'z',maybeinliers
#        test_points = data[test_idxs]
#        maybemodel = model.fit(maybeinliers)
#        test_err = model.get_error( test_points, maybemodel)
#        also_idxs = test_idxs[test_err < t] # select indices of rows with accepted points
#        alsoinliers = data[also_idxs,:]
#        if debug:
#            print 'test_err.min()',test_err.min()
#            print 'test_err.max()',test_err.max()
#            print 'np.mean(test_err)',np.mean(test_err)
#            print 'iteration %d:len(alsoinliers) = %d'%(
#                iterations,len(alsoinliers))
#        if len(alsoinliers) > d:
#            print np.asmatrix(maybeinliers), np.asmatrix(alsoinliers)
#            betterdata = np.concatenate( (maybeinliers, np.asmatrix(alsoinliers)) )
#            bettermodel = model.fit(np.asarray(betterdata))
#            better_errs = model.get_error( betterdata, bettermodel)
#            thiserr = np.mean( better_errs )
#            if thiserr < besterr:
#                bestfit = bettermodel
#                besterr = thiserr
#                print maybe_idxs, also_idxs
#                best_inlier_idxs = np.concatenate( (maybe_idxs, [also_idxs]) )
#        iterations+=1
#    if bestfit is None:
#        raise ValueError("did not meet fit acceptance criteria")
#    if return_all:
#        return bestfit, {'inliers':best_inlier_idxs}
#    else:
#        return bestfit
    iterations = 0
    bestfit = None
    besterr = np.inf
    best_inlier_idxs = None
    while iterations < k:
        #print data
        maybe_idxs, test_idxs = random_partition(n,data.shape[0])
        maybeinliers = data[maybe_idxs,:]
        test_points = data[test_idxs]
        maybemodel = model.fit(maybeinliers)
        test_err = model.get_error( test_points, maybemodel)
        also_idxs = test_idxs[test_err < t] # select indices of rows with accepted points
        alsoinliers = data[also_idxs,:]
        if debug:
            print 'test_err.min()',test_err.min()
            print 'test_err.max()',test_err.max()
            print 'np.mean(test_err)',np.mean(test_err)
            print 'iteration %d:len(alsoinliers) = %d'%(
                iterations,len(alsoinliers))
        if len(alsoinliers) > d:
            betterdata = np.concatenate( (maybeinliers, alsoinliers) )
            bettermodel = model.fit(betterdata)
            better_errs = model.get_error( betterdata, bettermodel)
            thiserr = np.mean( better_errs )
            if thiserr < besterr:
                bestfit = bettermodel
                besterr = thiserr
                best_inlier_idxs = np.concatenate( (maybe_idxs, also_idxs) )
        iterations+=1
    if bestfit is None:
        print "\n\n[ransac.py - line 152]"
        print "Ransac plane fitting did not meet fit accaptance criteria at current settings."
        print "Consider editing Ransac Parameters to be more generous or trying scan again."  
        print "This error often happens when no table is present in front of the robot.\n\n"
        import sys; 
        sys.exit()
        #Lets NOT raise an error. raise ValueError("did not meet fit acceptance criteria")
    if return_all:
        return bestfit, {'inliers':best_inlier_idxs}
    else:
        return bestfit


def random_partition(n,n_data):
    """return n random rows of data (and also the other len(data)-n rows)"""
    all_idxs = np.arange( n_data )
    np.random.shuffle(all_idxs)
    idxs1 = all_idxs[:n]
    idxs2 = all_idxs[n:]
    return idxs1, idxs2

class LinearLeastSquaresModel:
    """linear system solved using linear least squares

    This class serves as an example that fulfills the model interface
    needed by the ransac() function.
    
    """
    def __init__(self,input_columns,output_columns,debug=False):
        self.input_columns = input_columns
        self.output_columns = output_columns
        self.debug = debug
    def fit(self, data):
        A = np.vstack([data[:,i] for i in self.input_columns]).T
        B = np.vstack([data[:,i] for i in self.output_columns]).T
        x,resids,rank,s = scipy.linalg.lstsq(A,B)
        return x
    def get_error( self, data, model):
        A = np.vstack([data[:,i] for i in self.input_columns]).T
        B = np.vstack([data[:,i] for i in self.output_columns]).T
        B_fit = scipy.dot(A,model)
        err_per_point = np.sum((B-B_fit)**2,axis=1) # sum squared error per row
        print err_per_point
        return err_per_point
    
class PlaneLeastSquaresModel:
 
    def __init__(self, debug=False):
        self.debug = debug
    def fit(self, data):
        #print 'fit',data
        model = [data[0],np.cross(data[1] - data[0], data[2] - data[1])] #point, normal
        model[1] = model[1] / np.linalg.norm(model[1]) #normalize
        return model
    def get_error( self, data, model):
       
        #reject model if it's not horizontal
        max_angle = 30.0 * np.pi / 180.0
        angle = np.arccos(scipy.dot(np.array([0,0,1]),model[1].T)) #normal is normalized
        #print 'angle', angle / np.pi * 180.0
        
        if abs(angle) > max_angle:
            return np.ones(np.shape(data)[0]) * 999999999999999999999999999999
        #http://de.wikipedia.org/wiki/Hessesche_Normalform
        #print model[0], model[1]
        d = scipy.dot(model[0],model[1].T)
        #print 'd',d
        s = scipy.dot(data, model[1].T) - d
        #print 'dmds',data, model, d, 's',s
        #err_per_point = np.sum(np.asarray(s)**2, axis=1) # sum squared error per row
        #return err_per_point   
        return abs(s)
        
def test():
    # generate perfect input data

    n_samples = 500
    n_inputs = 1
    n_outputs = 1
    A_exact = 20*np.random.random((n_samples,n_inputs) )
    perfect_fit = 60*np.random.normal(size=(n_inputs,n_outputs) ) # the model
    B_exact = scipy.dot(A_exact,perfect_fit)
    assert B_exact.shape == (n_samples,n_outputs)

    # add a little gaussian noise (linear least squares alone should handle this well)
    A_noisy = A_exact + np.random.normal(size=A_exact.shape )
    B_noisy = B_exact + np.random.normal(size=B_exact.shape )

    if 1:
        # add some outliers
        n_outliers = 100
        all_idxs = np.arange( A_noisy.shape[0] )
        np.random.shuffle(all_idxs)
        outlier_idxs = all_idxs[:n_outliers]
        non_outlier_idxs = all_idxs[n_outliers:]
        A_noisy[outlier_idxs] =  20*np.random.random((n_outliers,n_inputs) )
        B_noisy[outlier_idxs] = 50*np.random.normal(size=(n_outliers,n_outputs) )

    # setup model

    all_data = np.hstack( (A_noisy,B_noisy) )
    input_columns = range(n_inputs) # the first columns of the array
    output_columns = [n_inputs+i for i in range(n_outputs)] # the last columns of the array
    debug = False
    model = LinearLeastSquaresModel(input_columns,output_columns,debug=debug)

    linear_fit,resids,rank,s = scipy.linalg.lstsq(all_data[:,input_columns],
                                                  all_data[:,output_columns])

    # run RANSAC algorithm
    ransac_fit, ransac_data = ransac(all_data,model,
                                     50, 1000, 7e3, 300, # misc. parameters
                                     debug=debug,return_all=True)
    if 1:
        import pylab

        sort_idxs = np.argsort(A_exact[:,0])
        A_col0_sorted = A_exact[sort_idxs] # maintain as rank-2 array

        if 1:
            pylab.plot( A_noisy[:,0], B_noisy[:,0], 'k.', label='data' )
            pylab.plot( A_noisy[ransac_data['inliers'],0], B_noisy[ransac_data['inliers'],0], 'bx', label='RANSAC data' )
        else:
            pylab.plot( A_noisy[non_outlier_idxs,0], B_noisy[non_outlier_idxs,0], 'k.', label='noisy data' )
            pylab.plot( A_noisy[outlier_idxs,0], B_noisy[outlier_idxs,0], 'r.', label='outlier data' )
        pylab.plot( A_col0_sorted[:,0],
                    np.dot(A_col0_sorted,ransac_fit)[:,0],
                    label='RANSAC fit' )
        pylab.plot( A_col0_sorted[:,0],
                    np.dot(A_col0_sorted,perfect_fit)[:,0],
                    label='exact system' )
        pylab.plot( A_col0_sorted[:,0],
                    np.dot(A_col0_sorted,linear_fit)[:,0],
                    label='linear fit' )
        pylab.legend()
        pylab.show()
        
        
def testPlane():



    debug = True
    model = PlaneLeastSquaresModel(debug)
    data = np.array([[0,0,0],[0,1,0],[0.1,12,0.1],[0,0,12],[1,0,0],[1,2,13]])
    # run RANSAC algorithm
    ransac_fit, ransac_data = ransac(data,model,
                                     3, 1000, 1, 2, # misc. parameters
                                     debug=debug,return_all=True)
    print ransac_fit
    print ransac_data
    
def testPlanePointcloud():
    import processor
    import configuration    

    cfg = configuration.configuration(TEST_FOLDER)
    #sc = scanner.scanner(cfg)
    pc = processor.processor(cfg)
    #pc.load_data('2009Oct30_162400')
    pc.load_data('2009Nov04_141226')
    pc.process_raw_data()
    
    
    debug = False
    model = PlaneLeastSquaresModel(debug)
    data = np.asarray(pc.pts3d_bound).T
    # run RANSAC algorithm
    ransac_fit, ransac_data = ransac(data,model,
                                     3, 1000, 0.02, 300, # misc. parameters
                                     debug=debug,return_all=True)
    print ransac_fit
    print ransac_data    
    print 'len inlier',len(ransac_data['inliers']),'shape pts',np.shape(pc.pts3d_bound)
    pc.pts3d_bound = pc.pts3d_bound[:,ransac_data['inliers']]
    pc.display_3d('height')

if __name__=='__main__':
    #testPlane()
    testPlanePointcloud()
