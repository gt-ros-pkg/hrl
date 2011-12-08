#!/usr/bin/python

## I'm less than enthused about the ("optimization") bloat in
## hrl/pfilter.  This implementation draws from that one, but
## eliminates a bunch of cruft and focuses on our particular details.

import random as rd
import numpy as np, math

class PFilter:
    def __init__(self, motion_model, measurement_model, particles):
        self.motion_model = motion_model  # class.  Provides 'predict' method: 1Dx2 particle -> 1Dx2 particle
        self.measurement_model = measurement_model # class.  Provides 'weight' method: 1Dx2 particle, measurement -> double

        # Particles. Should be np.matrix Nx3: [[X,Y,Weight],...]
        self.p = particles

    def motion( self, control_input ):
        print 'Motioning'
        new_xy = np.row_stack([ self.motion_model.predict( i[0:2] )  # for each [X,Y].T => new [X,Y].T
                                for i in self.p ])                   # stack them into Nx2
        new_p = np.column_stack([ new_xy, self.p[:,2] ]) # Particles keep weights => 3xN
        self.p = np.copy( new_p )
        return np.copy( new_p )

    def measurement( self, measurement ):
        # Takes in a single measurement, computes new weights for each
        # particle and combines them (multplicative) with the old weights.
        print 'Measuring.'
        w = np.array([ self.measurement_model.weight( measurement, i[0:2] ) for i in self.p ])
        new_w = self.p[:,2] * w

        new_p = np.column_stack([ self.p[:,0:2], new_w ])
        self.p = np.copy( new_p )
        return np.copy( new_p )

    def resample( self ):
        print 'Resampling'
        weighted_set = [ ( i[0:2], i[2] ) for i in self.p ]
        normalized_set = normalize_likelihood(weighted_set)
        
        new_xy = np.row_stack([ i for i in resample_uss( len(self.p), normalized_set )])
        new_p = np.column_stack([ new_xy, np.ones( new_xy.shape[0] ) ])
        self.p = np.copy( new_p )
        return np.copy( new_p )

    def step( self, control_input, measurement ):
        self.motion( control_input )
        self.measurement( measurement )
        self.resample()


def resample_uss(num_samples, particles):
    """ 
        Universal stochastic sampler (low variance resampling)
        num_samples - number of samples desired
        particles   - pairs of (state, weight) tuples
    """
    samples = []
    r        = rd.random() * (1.0 / float(num_samples))
    c        = (particles[0])[1]
    i        = 0

    for m in xrange(num_samples):
        U = r + m * (1.0 / float(num_samples))
        #print "U", U
        while U > c:
            i = i + 1
            if i >= len(particles):
                i = 0
            c = c + (particles[i])[1]
        samples.append((particles[i])[0])
    return samples

def normalize_likelihood(weighted_particles):
    """ Make all the particle weights sum up to 1 """
    def add(a,b):
        apart, aw = a
        bpart, bw = b
        return ('', aw+bw)
    total_weight = (reduce(add, weighted_particles, ('',0.0)))[1]
    def normalize(a):
        part, weight = a
        return (part, weight/total_weight)
    return map(normalize, weighted_particles)

class NoMotion:
    def __init__( self ):
        print 'Starting NoMotion.'

    def predict( self, p ):
        return p

class NoMeasure:
    def __init__( self ):
        print 'Starting NoMeasure.'

    def weight( self, p, m ):
        return 0.5



if __name__ == '__main__':

    X,Y = np.meshgrid( np.linspace(0,3,4),
                       np.linspace(0,3,4) )
    xyw = np.row_stack([ X.flatten(),  # Build Nx3
                         Y.flatten(),
                         np.ones( X.shape ).flatten() ]).T # weights (multiplicative)

    pf = PFilter( NoMotion(), NoMeasure(), xyw )
    pf.motion( 0 )
    pf.measurement( 0 )
    pf.resample()

    print pf.p

    pf.measurement( 0 )
    pf.p[0,2] = 5 # 10 times more likely
    pf.resample()

    print pf.p
