#! /usr/bin/python
import numpy as np, math
import scipy.io as spio
import roslib; roslib.load_manifest('hrl_object_fetching')
import rospy
import hrl_lib.util as hrl_util
from std_msgs.msg import Float64MultiArray, Bool
from pr2_msgs.msg import AccelerometerState, PressureState
from sensor_msgs.msg import JointState
from pr2_controllers_msgs.msg import JointTrajectoryControllerState

r_jt_idx_lis = [17, 18, 16, 20, 19, 21, 22]
l_jt_idx_lis = [29, 30, 28, 32, 31, 33, 34]
n_samples = 100 * 20
class Listener:
    def __init__(self):
        self.q = np.zeros(( n_samples, 7))
        self.t = np.zeros((n_samples, 1))
        self.tind = 0
        self.ind = 0

    def callback(self,msg):
        if self.ind % 10 == 0:
            print self.tind, self.ind
        for i,idx in enumerate(r_jt_idx_lis):
            self.q[self.ind, i] = msg.position[idx]
            print self.q[self.ind, i],
        print ""
        self.t[self.ind] = msg.header.stamp.to_sec()
        self.ind += 1
        

rospy.init_node('quick_data')
lis = Listener()
sub = rospy.Subscriber('/joint_states', JointState, lis.callback)
#sub_started = rospy.Subscriber('/grasper/grasp_executing', Bool, lis.grasp_executing)
rospy.spin()
hrl_util.save_pickle([lis.q[0:lis.ind,:], lis.t[0:lis.ind,:]], 'untuck_traj_6.pickle')
i = 0
while lis.q[i,4] != 0:
    print lis.q[i-1,3] - lis.q[i,3],
    i += 1
#c = 10
#data = {'q%d'%c:lis.q, 'des%d'%c:lis.des, 't%d'%c:lis.t}
#data = {'q':lis.q, 'des':lis.des, 't':lis.t}
#spio.savemat('grasp_err_all_1.mat', data)

