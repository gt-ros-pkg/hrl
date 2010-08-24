import roslib; roslib.load_manifest('hai_sandbox')
import rospy

import hrl_lib.util as ut
import hai_sandbox.pr2 as pr2
import pr2_msgs.msg as pm

import numpy as np
import time
import pdb
import sys

robot = None
should_switch = False
lmat0 = None
rmat0 = None
pressure_exceeded = False

def dict_to_arm_arg(d):
    trans = [d['pose']['position']['x'], 
             d['pose']['position']['y'], 
             d['pose']['position']['z']]
    rot = [d['pose']['orientation']['x'],
           d['pose']['orientation']['y'],
           d['pose']['orientation']['z'],
           d['pose']['orientation']['w']]
    return [trans, rot, d['header']['frame_id'], d['header']['stamp']]

def shutdown():
    global robot
    global should_switch
    if should_switch:
        rospy.loginfo('switching back joint controllers')
        robot.controller_manager.switch(['l_arm_controller', 'r_arm_controller'], ['l_cart', 'r_cart'])

def lpress_cb(pmsg):
    global lmat0
    global rmat0
    global pressure_exceeded
    lmat = np.matrix((pmsg.l_finger_tip)).T
    rmat = np.matrix((pmsg.r_finger_tip)).T
    if lmat0 == None:
        lmat0 = lmat
        rmat0 = rmat
        return

    lmat = lmat - lmat0
    rmat = rmat - rmat0
   
    #touch detected
    if np.any(np.abs(lmat) > 3000) or np.any(np.abs(rmat) > 3000):
        rospy.loginfo('Pressure limit exceedeD!! %d %d' % (np.max(np.abs(lmat)), np.max(np.abs(rmat))))
        pressure_exceeded = True


def imitate(data_fname):
    #    data = {'base_pose': pose_base, 
    #            'robot_pose': j0_dict,
    #            'arm': arm_used,
    #            'movement_states': None}
    global robot
    global should_switch
    global pressure_exceeded
    data = ut.load_pickle(data_fname)
    rospy.init_node('imitate')
    robot = pr2.PR2()
    rospy.Subscriber('/pressure/l_gripper_motor', pm.PressureState, lpress_cb)
    #rospy.Subscriber('/pressure/r_gripper_motor', pm.PressureState, self.lpress_cb)
    #self.pr2_pub = rospy.Publisher(pr2_control_topic, PoseStamped)
    state = 'init_manipulation'

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
        rospy.loginfo('switching controllers')
        robot.controller_manager.switch(['l_cart', 'r_cart'], ['l_arm_controller', 'r_arm_controller'])
        should_switch = True
        rospy.on_shutdown(shutdown)
        robot.left_arm.set_posture(robot.left_arm.POSTURES['elbowupl'])
        robot.right_arm.set_posture(robot.right_arm.POSTURES['elbowupr'])
        #rospy.loginfo('switching controllers sleeping..')
        #time.sleep(20)
        #rospy.loginfo('resuming')

        robot.left_arm.set_posture(robot.left_arm.POSTURES['elbowupl'])
        robot.right_arm.set_posture(robot.right_arm.POSTURES['elbowupr'])
        ## For each contact state
        for state in range(len(data['movement_states'])):
            if rospy.is_shutdown():
                break
            if pressure_exceeded:
                rospy.loginfo('Exiting movement state loop')
                break

            cur_state = data['movement_states'][state]
            rospy.loginfo("starting %s" % cur_state['name'])
            left_cart  = cur_state['cartesian'][0]
            right_cart = cur_state['cartesian'][1]
            start_time = cur_state['start_time']
            wall_start_time = rospy.get_rostime().to_time()

            for ldict, rdict in zip(left_cart, right_cart):
                if rospy.is_shutdown():
                    break
                if pressure_exceeded:
                    rospy.loginfo('Exiting inner movement state loop')
                    break
                lps = dict_to_arm_arg(ldict)
                rps = dict_to_arm_arg(rdict)

                msg_time_from_start = ((lps[3] - start_time) + (rps[3] - start_time))/2.0
                cur_time = rospy.get_rostime().to_time()
                wall_time_from_start = (cur_time - wall_start_time)

                sleep_time = (msg_time_from_start - wall_time_from_start) - .005
                if sleep_time < 0:
                    rospy.loginfo('sleep time < 0, %f' % sleep_time)

                if sleep_time > 0:
                    time.sleep(sleep_time)

                lps[3] = rospy.get_rostime().to_time()
                rps[3] = rospy.get_rostime().to_time()
                robot.left_arm.set_cart_pose(*lps)
                robot.right_arm.set_cart_pose(*rps)
            rospy.loginfo("%s FINISHED" % cur_state['name'])
            time.sleep(5)

        robot.controller_manager.switch(['l_arm_controller', 'r_arm_controller'], ['l_cart', 'r_cart'])
        should_switch = False
    
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
            lvel = np.column_stack(lvel)
            ltime = np.array(ltime) - cur_state['start_time']

            rarm = np.column_stack(rarm)
            rvel = np.column_stack(rvel)
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






