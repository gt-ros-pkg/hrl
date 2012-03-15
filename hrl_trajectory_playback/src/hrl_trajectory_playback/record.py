#! /usr/bin/python
import roslib
roslib.load_manifest('hrl_trajectory_playback')
import rospy

import hrl_lib.util as hrl_util
from sensor_msgs.msg import JointState

from collections import deque # fast appends
import numpy as np, math

# Hack (this code currently only supports the right arm!)
JOINT_NAMES = ['%s_shoulder_pan_joint', '%s_shoulder_lift_joint', '%s_upper_arm_roll_joint',
               '%s_elbow_flex_joint', '%s_forearm_roll_joint', '%s_wrist_flex_joint',
               '%s_wrist_roll_joint']

class Listener:
    def __init__(self, arm = 'right'):
        self.q = deque() # np.zeros(( n_samples, 7))
        self.t = deque() # np.zeros((n_samples, 1))
        self.arm = arm
        # self.tind = 0
        # self.ind = 0
        self.initialized = False
        self.starttime = rospy.Time(0)
        self.r_jt_idx_lis = None
        self.l_jt_idx_lis = None

    def callback( self, msg ):
        currtime = msg.header.stamp.to_sec()
        
        if not self.initialized:
            self.initialized = True
            self.starttime = currtime
            self.lasttime = currtime
            self.r_jt_idx_lis = [msg.name.index(joint_name % 'r') for joint_name in JOINT_NAMES]
            self.l_jt_idx_lis = [msg.name.index(joint_name % 'l') for joint_name in JOINT_NAMES]
            
        if currtime - self.lasttime > 1.0:
            self.lasttime = currtime
            print 'Still Capturing (%d sec)' % (currtime - self.starttime)


        if self.arm == 'right':
            q_ja = [ msg.position[idx] for idx in self.r_jt_idx_lis ]
        else:
            q_ja = [ msg.position[idx] for idx in self.l_jt_idx_lis ]
        self.q.append( q_ja )
        
        # for i,idx in enumerate( r_jt_idx_lis ):  # only dealing with right arm!
        #     self.q[self.ind, i] = msg.position[idx]
        #     print self.q[self.ind, i],
        # print ""
        # self.t[self.ind] = msg.header.stamp.to_sec()
        # self.ind += 1
        self.t.append([ currtime ])
        

if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--pkl', action='store', type='string', dest='pkl',
                 help='Output file [default=\'out.pkl\']', default='out.pkl')
    p.add_option('--left', action='store_true', dest='left_arm',
                 help='Use the left arm? [Right is default]')
    opt, args = p.parse_args()
    
    rospy.init_node('quick_data')

    if opt.left_arm:
        lis = Listener( arm = 'left' )
    else:
        lis = Listener( arm = 'right' )
    sub = rospy.Subscriber( '/joint_states', JointState, lis.callback )

    print 'Recording...  <Ctrl-C> to stop.'
    rospy.spin()
    sub.unregister()

    
    #hrl_util.save_pickle([lis.q[0:lis.ind,:], lis.t[0:lis.ind,:]], 'untuck_traj_6.pickle')
    print 'Outputting to: %s' % opt.pkl
    hrl_util.save_pickle([ np.array(lis.q),   # Nx7 numpy array
                           np.array(lis.t) ], # Nx1 numpy array
                         opt.pkl )
