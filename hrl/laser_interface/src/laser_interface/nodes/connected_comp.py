import numpy as np
import math
import util as ut
import itertools as it
import fun

URG_MAX_LASER_RANGE = 3.95
URG_ANGLE_START     = -2.006
URG_ANGLE_STEP      = 0.0061359233222901821
URG_NUM_SCANS       = 655

def connected_comps(scan, max_laser_range = URG_MAX_LASER_RANGE, max_obj_dist = 0.04, log=ut.log("connected_comps")):
    """ 
        Finds connected components in laser ranges similar to UrgScan except
          1) does not consider membership in eulidean coordinates but 
             using the ranges in polar coordinate
          2) this is a little more flexible as it also return indices in 
             the scan that points came from

        NOTE: returns a lazy list!
    """
    cur_ranges  = []
    cur_indices = []
    cluster_number = 0

    for idx in xrange(scan.shape[1]):
        if scan[0, idx] < max_laser_range:
            if len(cur_ranges) <= 0:
                cur_ranges.append(scan[0, idx])
                cur_indices.append(idx)
            else:
                r    = cur_ranges[-1]
                lidx = cur_indices[-1]
                if abs(scan[0, idx] - r) < max_obj_dist:
                    cur_ranges.append(scan[0, idx])
                    cur_indices.append(idx)
                else:
                    #Don't report noise components
                    if len(cur_ranges) > 2:
                        #log.log(lg.DEBUG, "#", cluster_number, "cluster", cur_indices)
                        cluster_number = cluster_number + 1
                        yield (cur_ranges, cur_indices)
                    cur_ranges  = []
                    cur_indices = []
        else:
            if len(cur_ranges) > 2:
                #log.log(lg.DEBUG, "#", cluster_number, "cluster", cur_indices)
                cluster_number = cluster_number + 1
                yield (cur_ranges, cur_indices)
            cur_ranges  = []
            cur_indices = []


def vec_frobenius_norm(vecs):
    return np.power(np.sum(np.power(vecs, 2), axis=0), 0.5)[0,:]


def cluster_distance(c1, c2):
    if len(c1) == 0 or len(c2) == 0:
        raise ValueError("distance metric undefined for empty clusters")

    dist = float('inf')
    for p1 in c1:
        for p2 in c2:
            d = np.linalg.norm(p1 - p2)
            if d < dist:
                dist = d

    return dist


def merge_clusters(clusters, MAX_SEP):
    for idx, c in enumerate(clusters):
        other_clusters = list(clusters)
        other_clusters.pop(idx)
        def close_enough(cluster):
            return cluster_distance(c, cluster) < MAX_SEP
        close, far = fun.split(close_enough, other_clusters)
        if len(close) > 0:
            new_center = fun.flatten(close)
            far.append(new_center)
            return far
    return clusters


def connected_fast(scan, max_laser_range = URG_MAX_LASER_RANGE, 
                   max_obj_dist = 0.04, log=ut.log("connected_comps")):
    components = connected_comps(scan, max_laser_range=max_laser_range, 
                                 max_obj_dist=max_obj_dist, log=log)
    def to_euclid(c):
        ranges, indices = c
        return list(ut.list_of_mat(euclid_of_laser(scan, indices=np.matrix(indices))))

    clusters = map(to_euclid, components)
    diff = 1
    while diff > 0:
        mclusters = merge_clusters(clusters, max_obj_dist)
        print len(mclusters)
        diff = len(clusters) - len(mclusters)
        if diff < 0:
            print "something is wrong in connected_comps_euclid"
        clusters = mclusters
    return clusters


def euclid_of_laser_mat(scans, angle_start = URG_ANGLE_STEP, 
        angle_step = URG_ANGLE_STEP, num_scans = URG_NUM_SCANS):
    """
        Gives euclidean coordinate equivalent for laser scans
    """
    indices = np.matrix(range(num_scans))
    angles         = ((indices * angle_step) + angle_start)
    x = np.multiply(scans, np.cos(angles))
    y = np.multiply(scans, np.sin(angles))
    return (x,y)


def euclid_of_laser(scans, indices=None, angle_start = URG_ANGLE_START, 
                    angle_step = URG_ANGLE_STEP, num_scans=URG_NUM_SCANS):
    """
         Takes laser ranges and returns euclidean ego centric
         coordinates of points, where +x is forward +y is left
         consistent with Player/Stage driving coordinates.
         (Assume laser scanner is flipped upside down).

         By default this node uses URG parameters, where points
         are scanned from left to right.  For SICK or other 
         scanners supply the angle_start and angle_step arguments.
    """

    if indices == None:
        indices = np.matrix(range(num_scans))
    else:
        if indices.__class__ != np.matrix:
            raise Exception("Argument indices must be a matrix")

    angles         = ((indices * angle_step) + angle_start)

    selected_scans = scans[0, indices]
    euclid         = ut.cart_of_pol(np.vstack((selected_scans, angles)))
    euclid[1,:]    = euclid[1,:] * -1.0
    return euclid


def laser_of_euclid(points):
    """
       Take a matrix of points and return matrix of scans (1xn)
    """
    return ut.norm(points)


def euclid_of_laser2(scans, indices=None, angle_start = URG_ANGLE_START, 
                    angle_step = URG_ANGLE_STEP, num_scans=URG_NUM_SCANS):
    """
	     NOTE: USE THIS IN THE FUTURE NOT ABOVE

         By default this node uses URG parameters, where points
         are scanned from left to right.  For SICK or other 
         scanners supply the angle_start and angle_step arguments.

         Return points in native urg frame
            +x is forward
            +y is right
    """

    if indices == None:
        indices = np.matrix(range(num_scans))
    else:
        if indices.__class__ != np.matrix:
            raise Exception("Argument indices must be a matrix")

    angles         = ((indices * angle_step) + angle_start)

    selected_scans = scans[0, indices]
    euclid         = ut.cart_of_pol(np.vstack((selected_scans, angles)))
    return euclid


def angleToIndex(angle):
	"""
	    angle in radians
	"""
	return int(angle / URG_ANGLE_STEP) + int(URG_NUM_SCANS / 2.0)


def range2indices(a1, a2):
	if a1 > a2:
		t = a1
		a1 = a2
		a2 = t
	i1 = angleToIndex(a1)
	i2 = angleToIndex(a2)
	number_pts = i2 - i1
	return np.matrix(range(number_pts)) + i1



















#def connected_comps_euclid(scans, max_laser_range = URG_MAX_LASER_RANGE, 
#        max_separation_dist = 0.05, log=ut.log("connected_comps")):
#    vrow, vcol   = np.where(scans < 3.99)
#    valid_points = euclid_of_laser(scans, indices=vcol)
#    def to_list(c):
#        return [c]
#    clusters     = map(to_list, list(ut.list_of_mat(valid_points)))
#
#    diff = 1
#    while diff > 0:
#        mclusters = merge_clusters(clusters, max_separation_dist)
#        diff = len(clusters) - len(mclusters)
#        if diff < 0:
#            print "something is wrong in connected_comps_euclid"
#        clusters = mclusters
#    return clusters
