#!/usr/bin/python

import numpy as np, math
import cPickle as pkl
import yaml
import scipy.stats as stats

class RfidModel:
    def __init__( self, yaml_fname ):
        # yaml_fname is yaml file something like: cap_360/rad_plot_combined.yaml
        # you will need to have run: ./../process_radpat_plots.py --yaml rad_plot_combined.yaml
        # which generates rad_plot_combined_MODEL.pkl

        # Loading the YAML file that specified how the data is to be processed.
        # This file was used to generate the 'use_combined' data set.
        
        f = open( yaml_fname )
        yaml_config = yaml.load( f )
        f.close()


        # Loading condensed data
        
        f = open( yaml_config['use_combined'].replace('.pkl','_MODEL.pkl'), 'r' )
        self.model = pkl.load( f )
        f.close()

        self.default_detect = 0.0  # These will be multiplied.  Will be zero outside the model range.
        self.default_rssi = 1.0

        self.num = 1

        


    def lookup( self, x, y ):
        # Note: The xind and yind are flipped for the model ind, since
        # the edges are defined from the histogram ( which is
        # transposed compared to the models )
        xind = np.sum( x > self.model['xedges'] ) - 1
        yind = np.sum( y > self.model['yedges'] ) - 1

        # Outside the bounds of our model?  Return no reads.
        if xind < 0 or xind >= len(self.model['xedges'])-1 or yind < 0 or yind >= len(self.model['yedges'])-1:
            return ( None, None, None )

        # Return value from models.
        return ( self.model['detect_model'][yind,xind],
                 self.model['rssi_model'][yind,xind],
                 self.model['stddev_model'][yind,xind] )

    def sample( self, x, y ):
        # Returns a sample (eg. detection yes or no, and RSSI)
        detect, rssi, stddev = self.lookup( x, y )
        if (not detect) or (not rssi) or (not stddev):
            return -1  # No read RSSI

        detected = np.random.uniform() <= detect  # all the mass below the detection thresh
        if not detected:
            return -1  # Probability

        return np.clip( np.random.normal( loc=rssi, scale=stddev ),
                        np.min( self.model['rssi_model'] ),
                        np.max( self.model['rssi_model'] )) # Bound the results by the model bounds
        

    def prob( self, x, y, measurement ):
        # Returns the detection and RSSI probabilities given the data-driven model
        detect, rssi, stddev = self.lookup( x, y )
        if (not detect) or (not rssi) or (not stddev):
            return self.default_detect, self.default_rssi

        rv = stats.norm( loc = rssi, scale = stddev )
        return detect, rv.pdf( measurement )
        

    def weight( self, measurement, particle ):
        # particle is 1Dx2: X,Y.  Measurement is RSSI
        detect_prob, rssi_prob = self.prob( particle[0], particle[1], measurement )
        return detect_prob * rssi_prob

    

    def weight_set( self, measurement, particle_set ):
        self.num += 1
        if self.num % 10 == 0:
            print '\tProcessing sample %d' % self.num
            
        xy = particle_set[:,0:2].T  # 2xN

        # Will automatically snap all particles to nearest bin (will fix below)
        bins_ind_x = np.sum( xy[0][:,np.newaxis] > self.model['xedges'][:-1], axis = 1 ) - 1
        bins_ind_y = np.sum( xy[1][:,np.newaxis] > self.model['yedges'][:-1], axis = 1 ) - 1

        # When the particle is outside the bin edges, lookup => (None, None, None), so
        #    detect_prob = 0.  Thus, handle these cases by setting weight to 0.0
        ind_x_less = np.where( xy[0] < self.model['xedges'][0] )[0]
        ind_x_more = np.where( xy[0] > self.model['xedges'][-1] )[0]
        ind_y_less = np.where( xy[1] < self.model['yedges'][0] )[0]
        ind_y_more = np.where( xy[1] > self.model['yedges'][-1] )[0]

        # Lookup values from model
        detect = self.model['detect_model'][bins_ind_y,bins_ind_x] # bins are flipped from histogram
        rssi = self.model['rssi_model'][bins_ind_y,bins_ind_x] # bins are flipped from histogram
        stddev = self.model['stddev_model'][bins_ind_y,bins_ind_x] # bins are flipped from histogram

        # Detection prob = model + uniform
        uniform_detect = 0.2 
        
        detect_prob = detect.filled( 0.0 )  # for masked values, we assign a zero probability
        detect_prob += uniform_detect

        detect_prob[ ind_x_less ] = uniform_detect # When outside the bin edges, assume detect prob is 0.0
        detect_prob[ ind_x_more ] = uniform_detect # When outside the bin edges, assume detect prob is 0.0
        detect_prob[ ind_y_less ] = uniform_detect # When outside the bin edges, assume detect prob is 0.0
        detect_prob[ ind_y_more ] = uniform_detect # When outside the bin edges, assume detect prob is 0.0

        detect_prob = np.clip( detect_prob, 0.0, 1.0 ) # force it to be a legit probability (since we're adding two distributions!)
        

        # RSSI prob is gaussian at each cell
        uniform_rssi = 0.2
        
        rssi_prob = 1.0 / np.sqrt(2.0 * np.pi * np.power(stddev,2.0))
        rssi_prob *= np.exp( -1.0 * np.power( measurement - rssi, 2.0 ) / (2.0*np.power(stddev,2.0)))

        rssi_prob = rssi_prob.filled( 0.0 ) # for masked values, we assign a zero probability
        rssi_prob += uniform_rssi

        rssi_prob[ ind_x_less ] = uniform_rssi # When outside the bin edges, assume detect prob is 0.0
        rssi_prob[ ind_x_more ] = uniform_rssi # When outside the bin edges, assume detect prob is 0.0
        rssi_prob[ ind_y_less ] = uniform_rssi # When outside the bin edges, assume detect prob is 0.0
        rssi_prob[ ind_y_more ] = uniform_rssi # When outside the bin edges, assume detect prob is 0.0

        rssi_prob = np.clip( rssi_prob, 0.0, 1.0 ) # force it to be a legit probability (since we're adding two distributions!)

        

        # Weight is multiplication of the two
        weight = detect_prob * rssi_prob


        # Setting the probability to 0.0 is harsh (it kills any later updates).
        # Add a small (uniform) distribution to account for this.
        
        # minw = np.min( weight[np.where( weight > 1e-20 )] ) * 0.9
        # weight = np.clip( weight, minw, np.max( weight ))

        # Update particle_set in-place
        particle_set[:,2] *= weight
        # particle_set[:,2] = np.clip( particle_set[:,2], 1e-10, np.max(particle_set[:,2]) )
        

        # Normalize so that the sum is 1.0
        particle_set[:,2] /= np.sum( particle_set[:,2] )

        return np.copy( particle_set )
        
yaml_fname = '/home/travis/svn/robot1/src/projects/rfid_datacapture/src/rfid_datacapture/rad_pattern_cap/rad_plot_shoulder_table_both_SpectrMedBot.yaml'
# yaml_fname = '/home/travis/svn/robot1/src/projects/rfid_datacapture/src/rfid_datacapture/cap_360/rad_plot_shoulder_left_datacap2.yaml'
# yaml_fname = '/home/travis/svn/robot1/src/projects/rfid_datacapture/src/rfid_datacapture/cap_360/rad_plot_combined.yaml'
#yaml_fname = '/home/travis/svn/robot1/src/projects/rfid_pf/src/rfid_pf/pencil_beam_model.yaml'

class NoMotion:
    def __init__( self ):
        print 'Starting NoMotion.'

    def predict_set( self, control, p_set ):
        return p_set



if __name__ == '__main__':
    import time

    X,Y = np.meshgrid( np.arange(-10,10,0.1), np.arange(-10,10,0.1))
    xyw = np.row_stack([ X.flatten(), Y.flatten(), np.ones( X.shape ).flatten() ]).T
    def test1( rm, measurement, particles ):
        t0 = time.time()

        w = np.array([ rm.weight( measurement, p[0:2] ) for p in particles ])
        rv = np.column_stack([ particles[:,0:2], w ])

        td = time.time() - t0
        return rv, td

    def test2( rm, measurement, particles ):
        t0 = time.time()

        rv = rm.weight_set( measurement, particles )

        td = time.time() - t0
        return rv, td

    rm = RfidModel( yaml_fname )

    print 'Starting 1'
    r1,t1 = test1( rm, 80, xyw )
    print 'Done 1 in %2.2f sec.' % t1

    print 'Starting 2'
    r2,t2 = test2( rm, 80, xyw )
    print 'Done 2 in %2.2f sec.' % t2

    print 'Speedup: %3.2fx' % (t1 / t2)
