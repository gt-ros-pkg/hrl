import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import hrl_lib.util as hru
import pylab as pb
import numpy as np
import itertools as it
import hrl_lib.rutils as ru
import sys
from cv_bridge.cv_bridge import CvBridge, CvBridgeError
import scipy.spatial as sp
import cv

##
# @return mat, mat, array
def contact_mat(contact_msgs):
    #start_time = contact_msgs[0].header.stamp.to_time()
    times = np.array([c.header.stamp.to_time() for c in contact_msgs]) #- start_time
    left, right = zip(*[[list(c.l_finger_tip), list(c.r_finger_tip)] for c in contact_msgs])
    
    left = np.matrix(left).T
    right = np.matrix(right).T
    return left, right, times


##
# @return array, array
def find_contact_times(left_mat, right_mat, times):
    left_mat = left_mat - left_mat[:, 0] 
    right_mat = right_mat - right_mat[:,0]
    
    #When/where did contact happen? 
    #TODO: we are assuming just one finger of one arm here!
    loc_r, time_c = np.where(np.abs(left_mat) > 250)
    times_contact = times[time_c.A1]

    return loc_r, times_contact


fname = sys.argv[1]
press_lt = '/pressure/l_gripper_motor'
press_rt = '/pressure/r_gripper_motor'
forearm_cam = '/l_forearm_cam/image_rect_color'

print 'reading pressure messages'
#Get the pressure messages
msgs_dict = ru.bag_sel(fname, [press_lt, press_rt])

#Get the image times
print 'getting image times'
forearm_cam_times = np.array([msg.header.stamp.to_time() for top, msg, t in ru.bag_iter(fname, [forearm_cam])])

print 'processing'
press_lmsgs = [msg for top, msg, t in msgs_dict[press_lt]]
press_rmsgs = [msg for top, msg, t in msgs_dict[press_rt]]

#ll_mat contains (contact_loc, contact_times)
ll_mat, lr_mat, times_l = contact_mat(press_lmsgs)
rl_mat, rr_mat, times_r = contact_mat(press_rmsgs)
contact_loc, times_contact_pressure = find_contact_times(ll_mat, lr_mat, times_l)
print 'contact loc', contact_loc

#figure out which images are closest in time
#note: each row is an instance in KDTrees, query return ([distance], [indices])
print 'finding closest images'
forearm_times_tree = sp.KDTree(np.matrix(forearm_cam_times).T)
closest_times_in_forearm_frames = [forearm_cam_times[forearm_times_tree.query([a_time])[1]] for a_time in times_contact_pressure]

def get_closest_msgs(fname, topics, times):
    times_set = set(times)
    for top, msg, t in ru.bag_iter(fname, topics):
        msg_time = msg.header.stamp.to_time()
        if len(times_set.intersection([msg_time])) > 0:
            yield msg

print 'getting & saving images'
bridge = CvBridge()
for ros_msg in get_closest_msgs(fname, [forearm_cam], closest_times_in_forearm_frames):
    msg_time = ros_msg.header.stamp.to_time() - forearm_cam_times[0]
    cv_image = bridge.imgmsg_to_cv(ros_msg, 'bgr8')
    cv.SaveImage("%.3f_touched.png" % msg_time, cv_image)

print 'plotting'
#Plot readings
pb.figure()
for i in range(ll_mat.shape[0]):
    pb.plot(times_l, ll_mat[i,:].T.A1, label=str(i))
pb.legend()
pb.show()



