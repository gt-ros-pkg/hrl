#! /usr/bin/python
import roslib
roslib.load_manifest('hrl_trajectory_playback')
import rospy

import pr2_controllers_msgs.msg as pm
from motion_planning_msgs.srv import FilterJointTrajectory
import hrl_lib.util as hrl_util
from hrl_pr2_lib.pr2 import PR2, Joint
from hrl_trajectory_playback.srv import TrajPlaybackSrv, TrajPlaybackSrvRequest

import numpy as np, math

class TrajPlayback():
    def __init__( self, name, fname ):
        self.name = name

        rospy.logout( 'TrajPlayback: Initializing (%s)' % self.name )
        try:
            rospy.init_node('traj_playback_'+self.name)
        except:
            pass

        dat = hrl_util.load_pickle( fname )
        self.q = np.mat( dat[0].T )

        # subsample
        self.q = self.q[:,0::30]
        # smooth
        self.t = np.linspace( 1.3, 1.3 + (self.q.shape[1]-1)*0.3, self.q.shape[1] ) 

        self.vel = self.q.copy()
        self.vel[:,1:] -= self.q[:,0:-1]
        self.vel[:,0] = 0
        self.vel /= 0.3

        self.pr2 = PR2()
        # rospy.wait_for_service('trajectory_filter/filter_trajectory')
        # self.filter_traj = rospy.ServiceProxy('trajectory_filter/filter_trajectory',
        #                                       FilterJointTrajectory)

        self.__service = rospy.Service( 'traj_playback/' + name,
                                        TrajPlaybackSrv,
                                        self.process_service )

        rospy.logout( 'TrajPlayback: Ready for service requests (%s)' % self.name )
        

    def process_service( self, req ):
        if req.play_backward:
            rospy.loginfo( 'TrajPlayback: Playing Reverse (%s)' % self.name )
        else:
            rospy.loginfo( 'TrajPlayback: Playing Forward (%s)' % self.name )
        
        if req.play_backward:
            tq = self.q[:,::-1].copy()
            tvel = -1 * self.vel[:,::-1].copy()
            tvel[:,-1] = 0
            tt = self.t.copy()
            tt[0] = 0
            tt[1:] += 0.1
        else:
            tq = self.q.copy()
            tvel = self.vel.copy()
            tt = self.t.copy()

        joint_traj = self.pr2.right._create_trajectory( tq, tt, tvel )
        dur = rospy.Duration.from_sec( tt[-1] + 5 )

        # result = self.filter_traj( trajectory = joint_traj,
        #                            allowed_time = dur)
        # ft = result.trajectory
        # ft.header.stamp = rospy.get_rostime() + rospy.Duration( 1.0 )

        g = pm.JointTrajectoryGoal()
        g.trajectory = joint_traj  # why not ft...?
        self.pr2.right.client.send_goal( g )
        self.pr2.right.client.wait_for_result()

        return True
        
if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--pkl', action='store', type='string', dest='pkl',
                 help='Output file [default=\'out.pkl\']', default='out.pkl')
    p.add_option('--name', action='store', type='string', dest='name',
                 help='Service "name": /traj_playback/name [default=\'test\']',
                 default='test')
    p.add_option('--play', action='store_true', dest='play',
                 help='Just play it once instead of building service [default=False]',
                 default=False)
    opt, args = p.parse_args()

    tp = TrajPlayback( opt.name, opt.pkl )
    
    if opt.play:
        req = TrajPlaybackSrvRequest()
        req.play_backward = 0
        tp.process_service( req )
    else:
        rospy.spin()
    
