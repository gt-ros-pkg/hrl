##############################################################################
##                Module for standard units and transforms in 2D
##############################################################################
import numpy
import util as ut
from StringIO import StringIO
import math
import opencv as cv

def rot_mat(a):
    """ 
        Makes a homogeneous rotation matrix that rotates the coordinate system 
        counter cw
        i.e. rotates points clockwise
    """
    return numpy.matrix([[math.cos(a),    math.sin(a), 0],
                         [-1*math.sin(a), math.cos(a), 0],
                         [0,              0,           1]])


def transform2D(o_p, o_angle):
    """ 
        Takes as input o_P_* and angle, both descriptions of the new coordinate
        frame in the original frame (frame o).  Returns a transfrom *_T_o that 
        describes points in the new coordinate frame (frame *).  
        The transformation applies a translation then a rotation.
    """
    t = numpy.matrix(numpy.eye(3))
    t[0:2][:,2] = -1 * o_p
    return rot_mat(o_angle) * t     


def create_screen_transform(w, h):
    """
       Create a 2D screenTglobal transformation matrix
    """
    cv_origin = numpy.matrix([-w/2.0, -h/2.0]).T
    trans     = transform2D(cv_origin, 0)
    rot       = numpy.matrix([[0, -1, 0],
                         [-1, 0, 0],
                         [ 0, 0, 1]])
    return trans * rot


def scale(pts, s):
    return pts * s


def cvpoint_of_pt(pts):
	"""
	   Careful this returns a list
	"""
	def cvpoint(pt):
		return cv.cvPoint(int(round(pt[0,0])), int(round(pt[1,0])))
	return map(cvpoint, ut.list_of_mat(pts))


def screenTglobal(w, h, pix_scale, pts_global):
	"""
		Takes homogeneous points in player/stage frame to 
		open cv's display coordinates. 

		w            width of opencv window
		h            height of opencv window
		scale        pixels per unit of pts_global
		pts_global   2xn matrix
	"""
	cv_t_l  = create_screen_transform(w, h)
	pts_cpy = pts_global.copy()
	return ut.homo_to_point(cv_t_l * ut.point_to_homo(scale(pts_cpy, pix_scale)))


def globalTscreen(w, h, pix_scale, pts_screen):
	"""
	    Takes point in screen coordinates (2xn) and return points in global
		coordinates (2xn)
	"""
	l_t_cv      = numpy.linalg.inv(create_screen_transform(w,h))
	pts_cpy     = pts_screen.copy()
	transformed = l_t_cv * ut.point_to_homo(pts_cpy)
	points2d    = ut.homo_to_point(transformed)
	scaled      = scale(points2d, 1.0/pix_scale)
	return scaled


class Pose2D(object):
    """ Represents a pose with column vector for pos and radians for angle"""
    def __init__(self, x, y, a):
        self.pos   = numpy.matrix([x,y]).T
        self.angle = a
        
    #TODO probably this isn't the best way to combine S and R2 space differences
    def __sub__(self, other):
        return numpy.linalg.norm(self.pos - other.pos) - ut.standard_rad(self.angle - other.angle)

    def sub_pos(self, other):
        return numpy.linalg.norm(self.pos - other.pos)
    
    def __str__(self):
        p = StringIO()
        print >>p, "(pos = ", self.pos.T, ", angle = ", self.angle, ")", 
        s = p.getvalue()
        return s





























#def robot_to_cv_coor(vc, w, h):
#    """
#        Takes homogeneous points in player/stage frame to 
#        open cv's display coordinates. 
#        
#        vc 2x1 matrix
#        w  width of opencv window
#        h  height of opencv window
#    """
#    v = ut.point_to_homo(vc)
#    cv_origin = numpy.matrix([-w/2.0, -h/2.0]).T
#    trans = transform2D(cv_origin, 0)
#    rot   = numpy.matrix([[0, -1, 0],
#                         [-1, 0, 0],
#                         [ 0, 0, 1]])
#    cv_t_l = trans * rot
#    return cv_t_l * v

