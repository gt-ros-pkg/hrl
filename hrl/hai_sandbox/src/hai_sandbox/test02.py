import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import hrl_lib.util as hru
import pylab as pb
import numpy as np
import itertools as it

def contact_mat(contact_msgs)
    start_time = contact_msgs[0].header.stamp
    times = np.array([c.header.stamp for c in contact_msgs]) - start_time
    left, right = zip(*[[list(c.l_finger_tip), list(c.r_finger_tip)] for c in contact_msgs])
    
    left = np.matrix(left).T
    right = np.matrix(right).T
    return left, right, times


def find_contact_times(left_mat, right_mat, times):
    left_mat = left_mat - left_mat[:, 0] 
    right_mat = right_mat - right_mat[:,0]
    
    #When/where did contact happen? 
    #TODO: we are assuming just one finger of one arm here!
    loc_r, time_c = np.where(np.abs(left_mat) > 250)
    times_contact = times[time_c.A1]

    return loc_r, times_contact
   

contact_msgs = hru.load_pickle('contact.pkl')
left_mat, right_mat, times = contact_mat(contact_msgs)
contact_loc, times_contact = find_contact_times(left_mat, right_mat, times)

#Plot readings
pb.figure()
x = range(left.shape[1])
for i in range(left.shape[0]):
    pb.plot(times, right_mat[i,:].T.A1, label=str(i))
pb.legend()
pb.show()






