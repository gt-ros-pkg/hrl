import util as ut
import math as mt
import numpy as np
from StringIO import StringIO
import transforms2d as t2d
import opencv.cv as cv
import types

class RobotMotion:
    """ 
        Generates sample from robot motion model, has methods used in particle filter 
        Use:
           motion_var- motion_var object
             or
           rotvar
           transvar
    """
    def __init__(self, motion_var=None, rotvar=None, transvar=None):
        if motion_var!= None:
            self.motion_var = motion_var
        elif (rotvar != None) and (transvar != None):
            self.motion_var = motion_var(rotvar, transvar)

    def predict_partial(self, odometry):
        return get_odom_sample(self.motion_var, odometry)

    def predict(self, odometry, particle):
        f = get_odom_sample(self.motion_var, odometry)
        return f(particle)


def make_set(cov, start_state, num_particles):
    """ Init a bunch of particles to be exactly at start state """
    return get_odom_samples(cov, start_state, t2d.Pose2D(0.0, 0.0, 0.0), num_particles)


def make_set_gauss(cov, start_state, num_particles):
    """ Initialize a gaussian distribution around start state """
    sx   = start_state.pos[0]
    sy   = start_state.pos[1]
    #TODO: check this
    mean = np.concatentate((start_state, np.matrix([start_state.angle])), axis=0)
    def gen_pose(idx):
        sample = np.random.multivariate_normal(mean, cov) 
        return t2d.Pose2D(sample[0,0], sample[1,0], sample[2,0])
    return map(gen_pose, range(num_particles))

####################################################################
#      Functions for sampling over 2D robot poses 
####################################################################
class move2d:
    """ 
        Store 2d movements as 3 components
            initial rotation
            translation
            final rotation
    """
    def __init__(self, rot1, trans, rot2):
        self.rot1 = rot1
        self.trans = trans
        self.rot2 = rot2

    def __str__(self):
        s = StringIO()
        print >>s, "( rot1:", self.rot1, ", trans:" , 
        print >>s, self.trans, ", rot2:" , self.rot2, ")"
        return s.getvalue()

class motion_var:
    """ variances used for motion generation """
    def __init__(self, rotvar=(mt.radians(5), mt.radians(5)), transvar=(0.02, 0.03)):
        """
            rotvar   - a tuple of 2 floats
                       represents degree variance introduced per 1 rotation
                       degree variance per 1 meter traveled

            transvar - a tuple of 2 floats
                       distance error introduced per 1 rotation
                       distance error introduced per 1 meter traveled
        """
        self.rotvar   = rotvar
        self.transvar = transvar

def p2d_to_move2d(odom):
    """
        Decompose odometry readings into three components
            initial rotation
            translation
            final rotation
        odom - a differential reading between this and the last time step
    """
    if (odom.pos == np.matrix([0.0, 0.0]).T).all():
        orot1 = 0.0
    else:
        orot1 = ut.standard_rad(ut.ang_of_vec(odom.pos) - odom.angle) 

    otran = np.linalg.norm(odom.pos)
    orot2 = odom.angle - orot1 
    return move2d(orot1, otran, orot2)


def get_odom_samples(cov, s, motion, num_particles):
    """ Get a pose sample, use this function to get multiple samples from robot motion model """
    sampler = get_odom_sample(cov, motion)
    particles = []
    for i in xrange(num_particles):
        sample = sampler(s)
        particles.append(sample)
    return particles


def get_odom_sample(motion_variances, motion): #s, motion_variances):
    """ 
        Get a pose sample using odometry motion model (from Probabilistic Robotics p. 136)
        use this method to get a sample pose, ignore others 
        motion           - odometry in p2d format
        s                - state    in p2d format from time t-1
        motion_variances - motion variances
    
        returns a new p2d, a perturbed version of motion+s
    """
    u_move2d = p2d_to_move2d(motion)
    #Partially applied motion 
    def get_odom_sample_partial(s):
        # Sample 
        srot1 = sample_rot1 (u_move2d, motion_variances)
        trans = sample_trans(u_move2d, motion_variances)
        srot2 = sample_rot2 (u_move2d, motion_variances)

        rot1  = ut.standard_rad(u_move2d.rot1  - srot1)
        trans =                 u_move2d.trans - trans 
        rot2  =                 u_move2d.rot2  - srot2 

        #print mt.degrees(rot1), trans, mt.degrees(rot2)
        # Calculate new values 
        sx = s.pos[0,0]
        sy = s.pos[1,0]
        x = sx + trans * mt.cos(s.angle + rot1)
        y = sy + trans * mt.sin(s.angle + rot1)
        total_rot = ut.standard_rad(s.angle + rot1 + rot2) 
        return t2d.Pose2D(x,y, total_rot)
    return get_odom_sample_partial


def sample_rot1(odom, odom_cov):
    rotvar = odom_cov.rotvar 
    rotvar_0 = rotvar[0] / (np.pi * 2.0)
    scale  =  (rotvar_0 * abs(odom.rot1)) + (rotvar[1] * abs(odom.trans))

    if scale == 0.0:
        return 0.0
    else:
        return np.random.normal(scale=scale)


def sample_trans(odom, odom_cov):
    transvar   = odom_cov.transvar 
    rot_comp   = transvar[0] * abs(odom.rot1 + odom.rot2)
    trans_comp = transvar[1] * abs(odom.trans)
    scale      = rot_comp + trans_comp
    if scale == 0.0:
        return 0.0
    else:
        return np.random.normal(scale=scale)


def sample_rot2(odom, odom_cov):
    rotvar   = odom_cov.rotvar 
    rotvar_0 = rotvar[0] / (np.pi * 2.0)
    scale  = (rotvar_0 * abs(odom.rot2)) + (rotvar[1] * abs(odom.trans))
    if scale == 0.0:
        return 0.0
    else:
        return np.random.normal(scale=scale)


def draw_weighted_Pose2D(display, max_weight, particles):
    for p in particles:
        if type(p) is types.TupleType:
            part, weight = p
            rpos = part.pos
        else:
            part = p
            rpos = p.pos

        x = mt.cos(part.angle) * .07
        y = mt.sin(part.angle) * .07

        dir  = rpos.copy()
        dir[0,0] = dir[0,0] + x
        dir[1,0] = dir[1,0] + y

        pos  = display.to_screen(rpos)
        dirp = display.to_screen(dir)

        if type(p) is types.TupleType:
            color = round(255.0 * (weight/max_weight))
            cv.cvCircle(display.buffer, cv.cvPoint((int) (pos[0,0]), (int) (pos[1,0])), 
                        2, cv.cvScalar(255, 255-color, 255), cv.CV_FILLED, cv.CV_AA)
            cv.cvCircle(display.buffer, cv.cvPoint((int) (pos[0,0]), (int) (pos[1,0])), 
                        2, cv.cvScalar(200, 200, 200), 8, cv.CV_AA)
        else:
            cv.cvCircle(display.buffer, cv.cvPoint((int) (pos[0,0]), (int) (pos[1,0])), 
                        2, cv.cvScalar(150, 150, 150), cv.CV_FILLED, cv.CV_AA)

        cv.cvLine(display.buffer, cv.cvPoint((int) (pos[0,0]), (int) (pos[1,0])),
                                  cv.cvPoint((int) (dirp[0,0]), (int) (dirp[1,0])),
                                  cv.cvScalar(100,200,100), 1, cv.CV_AA, 0)

#class odometry2d:
#    """ an odometry reading """
#    def init(self, rot, trans):
#        self.rot = rot
#        self.trans = trans
#  type params = RobotSampler.cov 
#  and state = Pose2.t
#  and control = Pose2.t
#
#  let predict odom_cov u (* s *)=
#    let partial = RobotSampler.get_odom_sample u in
#      (fun s -> partial s odom_cov)
#
# Keep it simple for now and limit to 2D motion
#type state = Pose2.t
#type error_wts   = float*float
#type cov   = {rot1w: error_wts;
#              transw: error_wts;
#              rot2w: error_wts}

# Example covariance for a typical situation 
#let image_motion_cov = {rot1w=(0.995,0.005); transw=(0.995,0.005); rot2w=(0.995,0.005)}
#let make_cov rot1 trans rot2 = {rot1w=rot1; transw=trans; rot2w=rot2}


if __name__ == "__main__":
    import nodes as nd
    import detection_appearance as da

    rotvar   = (0.8, 0.2)
    transvar = (0.1, 0.9)
    motion_model = RobotMotion(motion_var(rotvar=rotvar, transvar=transfar))

    cov = np.eye(3)
    app_model    = DetectionAppearance()
    disp = nd.RobotDisp()





