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
from hrl_pr2_lib.pr2 import PR2, Joint
from motion_planning_msgs.srv import FilterJointTrajectory
import pr2_controllers_msgs.msg as pm

UNTUCK = False

rospy.init_node('untuck_grasp')
[q, t] = hrl_util.load_pickle('untuck_traj_6.pickle')
q = np.mat(q.T)
q = q[:,0::30]
#trim_val = -1
#q = q[:,0:trim_val]
#q[:,-1] = np.mat([[-0.269010636167, -0.307363010617, -1.52246181858, -1.67151320238, 294.057002545, -1.55558026761, -1.71909509593]]).T
vel = q.copy()
vel[:,1:] -= q[:,0:-1]
vel[:,0] = 0
vel /= 0.3
#vel[:,-1] = np.mat([[0.1] * 7]).T
t = np.linspace(1.3, 1.3 + (q.shape[1] - 1) * 0.3, q.shape[1])
#t[-1]  = t[-1] + 2
#t = q / vel
#t = t.max(1)
#t = np.cumsum(t)
#print t
pr2 = PR2()
rospy.wait_for_service('trajectory_filter/filter_trajectory')
filter_traj = rospy.ServiceProxy('trajectory_filter/filter_trajectory', FilterJointTrajectory)
#print q.shape
#vel = vel[:,0:trim_val]
#t = t[0:trim_val]
if not UNTUCK:
    q = q[:,::-1]
    vel = -vel[:,::-1]
    vel[:,-1] = 0
    #vel[:,0] = np.mat([[0.1] * 7]).T
    #t[-1] = t[-1] - 2
    t[0] = 0
    t[1:] += 0.1
    #print q
    #print vel

joint_traj = pr2.right._create_trajectory(q, t, vel)
result = filter_traj(trajectory=joint_traj, allowed_time=rospy.Duration.from_sec(20))
filtered_traj = result.trajectory
filtered_traj.header.stamp = rospy.get_rostime() + rospy.Duration(1.)
g = pm.JointTrajectoryGoal()
g.trajectory = joint_traj
pr2.right.client.send_goal(g)

#pr2.right.set_poses(q, t, vel)
#def set_poses(self, pos_mat, times, vel_mat=None, block=True):
