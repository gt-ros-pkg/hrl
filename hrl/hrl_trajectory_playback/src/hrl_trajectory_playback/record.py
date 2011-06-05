#! /usr/bin/python
import roslib
roslib.load_manifest('hrl_trajectory_playback')
import rospy

import hrl_lib.util as hrl_util
from sensor_msgs.msg import JointState

from collections import deque # fast appends
import numpy as np, math

# Hack (this code currently only supports the right arm!)
r_jt_idx_lis = [17, 18, 16, 20, 19, 21, 22]
l_jt_idx_lis = [29, 30, 28, 32, 31, 33, 34]

class Listener:
    def __init__(self, arm = 'right'):
        self.q = deque() # np.zeros(( n_samples, 7))
        self.t = deque() # np.zeros((n_samples, 1))
        self.arm = arm
        # self.tind = 0
        # self.ind = 0
        self.initialized = False
        self.starttime = rospy.Time(0)

    def callback( self, msg ):
        currtime = msg.header.stamp.to_sec()
        
        if not self.initialized:
            self.initialized = True
            self.starttime = currtime
            self.lasttime = currtime
            
        if currtime - self.lasttime > 1.0:
            self.lasttime = currtime
            print 'Still Capturing (%d sec)' % (currtime - self.starttime)

        if self.arm == 'right':
            q_ja = [ msg.position[idx] for idx in r_jt_idx_lis ]
        else:
            q_ja = [ msg.position[idx] for idx in l_jt_idx_lis ]
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
