import roslib; roslib.load_manifest('hai_sandbox')
import rospy

import hrl_lib.util as ut
import hai_sandbox.pr2 as pr2

import numpy as np
import time
import pdb
import sys

def dict_to_ps(d):
    ps = gm.PoseStamped()
    ps.pose.position.x = d['pose']['position']['x']
    ps.pose.position.y = d['pose']['position']['y']
    ps.pose.position.z = d['pose']['position']['z']
    ps.pose.orientation.x = d['pose']['orientation']['x']
    ps.pose.orientation.y = d['pose']['orientation']['y']
    ps.pose.orientation.z = d['pose']['orientation']['z']
    ps.pose.orientation.w = d['pose']['orientation']['w']
    ps.header.frame_id = d['header']['frame_id']
    ps.header.stamp = d['header']['stamp']
    return ps


def imitate(data_fname):
    #    data = {'base_pose': pose_base, 
    #            'robot_pose': j0_dict,
    #            'arm': arm_used,
    #            'movement_states': None}
    data = ut.load_pickle(data_fname)
    rospy.init_node('imitate')
    robot = pr2.PR2()
    #self.pr2_pub = rospy.Publisher(pr2_control_topic, PoseStamped)
    state = 'drive'

    ##Need to be localized!!
    ## NOT LEARNED: go into safe state.
    
    ## drive. learned locations. (might learn path/driving too?)
    if state == 'drive':
        t, r = data['base_pose']
        print t
        r = robot.base.set_pose(t, r, '/map', block=True)
        rospy.loginfo('result is %s' % str(r))
        state = 'init_manipulation'
    ## Need a refinement step
    
    ## Move joints to initial state. learned initial state. (maybe coordinate this with sensors?)
    #Put robot in the correct state
    if state == 'init_manipulation':
        rospy.loginfo('STATE init_manipulation')
        j0_dict = data['robot_pose']
        cpos = robot.pose()
        robot.left_arm.set_poses (np.column_stack([cpos['larm'], j0_dict['poses']['larm']]), np.array([0.1, 5.]), block=False)
        robot.right_arm.set_poses(np.column_stack([cpos['rarm'], j0_dict['poses']['rarm']]), np.array([0.1, 5.]), block=False)
        robot.head.set_poses(np.column_stack([cpos['head_traj'], j0_dict['poses']['head_traj']]), np.array([.01, 5.]))
        robot.torso.set_pose(j0_dict['poses']['torso'][0,0], block=True)
        state = 'manipulate'

    if state == 'manipulate_cart':
        rospy.loginfo('STATE manipulate')
        rospy.loginfo('there are %d states' % len(data['movement_states']))
        ## For each contact state
        for state in range(len(data['movement_states'])):
            cur_state = data['movement_states'][state]
            rospy.loginfo("starting %s" % cur_state['name'])
            left_cart  = cur_state['cartesian'][0]
            right_cart = cur_state['cartesian'][1]
            start_time = cur_state['start_time']

            for ldict, rdict in zip(left_cart, right_cart):
                lps = dict_to_ps(ldict)
                rps = dict_to_ps(rdict)
                time_from_start = ((lps.header.stamp - start_time) + (rps.header.stamp - start_time))/2.0
                cur_time = rospy.get_rostime()
                ntime = cur_time + rospy.Duration(time_from_start)

                diff_time = (ntime - rospy.get_rostime()).to_sec()
                if diff_time < 0:
                    rospy.logerror('DIFF time < 0, %f' % diff_time)
                time.sleep(diff_time - .005)
                #Publish...


    
    if state == 'manipulate':
        rospy.loginfo('STATE manipulate')
        rospy.loginfo('there are %d states' % len(data['movement_states']))
        ## For each contact state
        for state in range(len(data['movement_states'])):
            cur_state = data['movement_states'][state]
            rospy.loginfo("starting %s" % cur_state['name'])
    
            larm, lvel, ltime, rarm, rvel, rtime = zip(*[[jdict['poses']['larm'], jdict['vels']['larm'], jdict['time'], \
                                                          jdict['poses']['rarm'], jdict['vels']['rarm'], jdict['time']] \
                                                                for jdict in cur_state['joint_states']])
    
            larm = np.column_stack(larm)
            rarm = np.column_stack(rarm)
            lvel = np.column_stack(lvel)
            rvel = np.column_stack(rvel)
            ltime = np.array(ltime) - cur_state['start_time']
            rtime = np.array(rtime) - cur_state['start_time']
    
            ## send trajectory. wait until contact state changes or traj. finished executing.
            robot.left_arm.set_poses(larm[:,0], np.array([2.]), block=False)
            robot.right_arm.set_poses(rarm[:,0], np.array([2.]), block=True)
    
            robot.left_arm.set_poses(larm, ltime, vel_mat=lvel, block=False)
            robot.right_arm.set_poses(rarm, rtime, vel_mat=rvel, block=True)
    
            rospy.loginfo("%s FINISHED" % cur_state['name'])
            time.sleep(5)

    ## rosbag implementation steps in time and also figures out how long to sleep until it needs to publish next message
    ## Just play pose stamped back at 10 hz
    ## For each contact state

class ControllerTest:
    def __init__(self):
        pass

    def run(self):
        self.robot = pr2.PR2()
        rospy.loginfo('switching to cartesian controllers')
        self.robot.controller_manager.switch(['l_cart', 'r_cart'], ['l_arm_controller', 'r_arm_controller'])
        rospy.on_shutdown(self.shutdown)
        r = rospy.Rate(1)
        #publish posture & cartesian poses
        while not rospy.is_shutdown():
            self.robot.left_arm.set_posture(self.robot.left_arm.POSTURES['elbowupl'])
            self.robot.right_arm.set_posture(self.robot.right_arm.POSTURES['elbowupr'])
            r.sleep()

    def shutdown(self):
        rospy.loginfo('switching back joint controllers')
        self.robot.controller_manager.switch(['l_arm_controller', 'r_arm_controller'], ['l_cart', 'r_cart'])

    
if __name__ == '__main__':
    imitate(sys.argv[1])
    if False:
        c = ControllerTest()
        c.run()






