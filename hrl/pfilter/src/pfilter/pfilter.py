#!/usr/bin/python
import numpy as np
import random as rd

############################################################################
###               Functions implementing a particle filter
############################################################################
# Note:
#    To instantiate a particle filter you will need a motion and appearance
#    model.  Below are signatures and description of the motion and 
#    appearance models:
#        (optional) motion.make_set: (int)                -> list state
#        motion.predict:             (control, state)     -> state
#        appearance.weight:          (measurement, state) -> double
#
#    Where what is considered a 'state' must agree between the motion and 
#    appearance classes.
#
#    Optional:
#       The particle filter can be sped up by defining additional functions:
#          * weight_partial  - partial application
#          * weight_set      - any other optimizations
#
#          * predict_partial - partial application

def retTrue( *args ):
    return True

def retFalse( *args ):
    return False

class PFilter:
    def __init__(self, motion, appearance):
        """ Makes a particle filter """
        self.motion = motion
        self.appearance = appearance

    def step(self, control_input, measurement, particle_set, draw_func=None, set_op=False, should_resample_func=retTrue):
        """ Go through one cycle of particle filtering """
        #print particle_set
        predictions    = predict(self.motion, control_input, particle_set)
        #print predictions
        weighted_set   = likelihood(self.appearance, measurement, predictions)
        #print weighted_set
        if draw_func is not None:
            draw_func(weighted_set)

        if should_resample_func():
            if set_op:
                normalized_set = set_norm_likelihood(weighted_set)
                retVal = set_resample_uss(particle_set.shape[1], normalized_set)
            else:
                normalized_set = normalize_likelihood(weighted_set)
                retVal = resample_uss(len(particle_set), normalized_set)
        else:
            retVal = weighted_set # Change by travis to avoid resampling, but return weights as part of particle set
            
        return retVal

############################################################################
###                                Helpers
############################################################################
def predict(motion_model, control_input, particle_set):
    """ Predict using motion model """
    if hasattr(motion_model, "predict_partial"):
        f = motion_model.predict_partial(control_input)
        predictions = [f(particle) for particle in particle_set]
    elif hasattr(motion_model, "predict_set"):
        predictions = motion_model.predict_set(control_input, particle_set)    
    else:
        predictions = [motion_model.predict(control_input, particle) for particle in particle_set]
    return predictions


def likelihood(appearance_model, measurement, particle_set):
    """ Evaluate using appearance model """
    if hasattr(appearance_model, "weight_set"):
        weighted = appearance_model.weight_set(measurement, particle_set)
    elif hasattr(appearance_model, "weight_partial"):
        f = appearance_model.weight_partial(measurement)
        weighted = [(particle, f(particle)) for particle in particle_set]
    else:
        weighted = [(particle, appearance_model.weight(measurement, particle)) for particle in particle_set]
    return weighted

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

def set_resample_uss(num_samples, particles):
    """ 
        Universal stochastic sampler (low variance resampling)
        num_samples - number of samples desired
        particles   - pairs of (state, weight) tuples
    """
    ##[[ x1     x2     ...  ]
    ## [ y1     y2     ...  ]
    ## [ 0.     0.     ...  ]
    ## [ 1.     1.     ...  ]
    ## [ w1     w2     ... ]]

    samples = np.matrix( np.zeros( (4, num_samples) ))
    
    r        = rd.random() * (1.0 / float(num_samples))
    c        = particles[4,0]
    i        = 0

    for m in xrange(num_samples):
        U = r + m * (1.0 / float(num_samples))
        #print "U", U
        while U > c:
            i = i + 1
            if i >= particles.shape[1]:
                i = 0
            c = c + particles[4,i]
        samples[:,m] = particles[0:4,i]
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

def set_norm_likelihood(weighted_particles):
    wp = weighted_particles
    
    wSum = np.sum( wp[4,:] )
    wp[4,:] = wp[4,:] / wSum
    return wp
    


if __name__ == "__main__":
    particles     = [("4", 4), ("1",1), ("2",2), ("3", 3)]
    normalized    = normalize_likelihood(particles)
    #print normalized

    num_particles = 1000
    new_particles = resample_uss(num_particles, normalized)
    #print new_particles

    counts = {}
    for pair in particles:
        name, numb = pair
        counts[name] = 0

    for p in new_particles:
        counts[p] = 1 + counts[p]

    for k in counts.keys():
        print k, " :", (counts[k] / float(num_particles))


