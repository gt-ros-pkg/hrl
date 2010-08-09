import numpy as np
import util as ut
import prob as pr
import itertools as it
import types
import opencv.cv as cv

class DetectionAppearance:
    def __init__(self, cov):
        """ 
              Appearance model for tracking when there is a detector
              available that gives 2d poses of objects.
              cov             - uncertainty of measurements
              measurement     - 2x1 matrix
              state           - 2x1 matrix
        """
        self.cov = cov 
        #self.validation_prob = validation_prob

    def weight(self, measurement, particle):
        """ 
            measurement - 2D column numpy.matrix
            particle    - Pose2D 
        """
        gauss = pr.Gaussian(m = measurement, v = self.cov)
        f = gauss.pdf_mat()
        w = f(particle)
        return w[0]

    def weight_partial(self, measurement):
        gauss = pr.Gaussian(m = measurement, v = self.cov)
        f = gauss.pdf_mat()
        def weightp(particle):
            w = f(particle)
            return w[0]
        return weightp

    def weight_set(self, measurement, particle_set):
        pos_mat    = ut.list_mat_to_mat(particle_set, axis=1)
        gauss      = pr.Gaussian(m = measurement, v = self.cov)
        f          = gauss.pdf_mat()
        weight_mat = f(pos_mat)

        def pair_weights(tup):
            part, idx = tup
            return (part, weight_mat[idx])
        return map(pair_weights, it.izip(particle_set, xrange(len(particle_set))))



def draw_weighted_2D(display, max_weight, particles):
    for p in particles:
        if type(p) is types.TupleType:
            rpos, weight = p
        else:
            rpos = p

        pos  = display.to_screen(rpos)

        if type(p) is types.TupleType:
            color = round(255.0 * (weight/max_weight))
            cv.cvCircle(display.buffer, cv.cvPoint((int) (pos[0,0]), (int) (pos[1,0])), 
                        3, cv.cvScalar(255, 255-color, 255), cv.CV_FILLED, cv.CV_AA)
            cv.cvCircle(display.buffer, cv.cvPoint((int) (pos[0,0]), (int) (pos[1,0])), 
                        3, cv.cvScalar(200, 200, 200), 1, cv.CV_AA)
        else:
            cv.cvCircle(display.buffer, cv.cvPoint((int) (pos[0,0]), (int) (pos[1,0])), 
                        2, cv.cvScalar(150, 150, 150), cv.CV_FILLED, cv.CV_AA)


















    #def weight_matrix(self, measurement):
    #    gauss = pr.Gaussian(m = measurement, v = self.cov)
    #    f = gauss.pdf_mat()
    #    def weightp(particle):
    #        w = f(particle.pos)
    #        return w[0]
    #    return weightp



