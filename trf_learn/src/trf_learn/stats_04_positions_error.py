import roslib; roslib.load_manifest('trf_learn')
import rcommander_pr2_gui.pr2_utils as pru
import rcommander.graph_model as ghm
import rospy
import tf 

import hrl_lib.tf_utils as tfu
import geometry_msgs.msg as gm
import hrl_lib.util as ut
import os.path as pt
import glob
import time
import numpy as np
import math
import threading as thr
import copy

class PositionErrorExp:
    def __init__(self):
        ################################################################################
        #Which PR2 object to use??
        # HRL? RCommander? Perhaps the RCommander object needs to be released? We'll go
        # with the RCommander version and slowly refactor it out later... trf_learn
        # will need to have a dependency on RCommander.
        rospy.init_node('position_error', anonymous=True)
        self.tf_listener = tf.TransformListener()
        self.robot = pru.PR2(self.tf_listener)

        trf_dir = roslib.packages.get_pkg_dir('trf_learn')
        self.path_to_actions = pt.join(trf_dir, 'behaviors')
        self.pose_sub = rospy.Subscriber('/monty/pose', gm.TransformStamped, self.pose_cb)

        self.message_lock = thr.RLock()
        self.message = None
        self.messages = []

    def pose_cb(self, msg):
        if self.message_lock.acquire(True):
            self.message = msg
            self.messages.append(msg)
            self.message_lock.release()

    def turn_to_point(self, point_bl, block=True):
        ang = math.atan2(point_bl[1,0], point_bl[0,0])
        rospy.loginfo('turn_to_point: turning by %.2f deg' % math.degrees(ang))
        self.robot.base.turn_by(-ang, block=block, overturn=True)

    #Copied over from application behaviors
    def move_base_planner(self, trans, rot):
        p_bl = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), np.matrix(trans).T)
        self.turn_to_point(p_bl)
        rvalue = self.robot.base.set_pose(trans, rot, '/map', block=True)
        print 'planner return value', rvalue
        #if rvalue == 4:
        #    print 'planner failed exiting!'
        #    exit()
        p_bl = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), np.matrix(trans).T)
        self.robot.base.move_to(p_bl[0:2,0], True)
        t_end, r_end = self.robot.base.get_pose()
        return rvalue==3, np.linalg.norm(t_end[0:2] - np.array(trans)[0:2])

    def run(self):
        done = False
        while not done:
            value = raw_input("Do you need the robot to execute a behavior? y/n\n")
            if value.lower() == 'n':
                done = True
                break

            behaviors = glob.glob(pt.join(self.path_to_actions, '*'))
            for idx, p in enumerate(behaviors):
                print idx, pt.split(p)[1]
            picked = int(raw_input("Select a behavior to execute\n"))

            #0. Load RCommander file, raise arm to pose, say, sleep, grip.
            self.graph_model = ghm.GraphModel.load(behaviors[picked])
            state_machine = self.graph_model.create_state_machine(self.robot)
            outcome = state_machine.execute()
            
        
        #1. Record two poses from map, a and b.
        raw_input("I'm going to record the current position. Press enter to continue.\n")
        a_loc = self.robot.base.get_pose()

        raw_input("Move the robot to the next position. Press enter to continue.\n")
        b_loc = self.robot.base.get_pose()

        #2. Move to a, record pose, move to b record current pose.
        tstring = time.strftime('%A_%m_%d_%Y_%I_%M%p')
        locations = [a_loc, b_loc]
        base_poses = [[], []]
        for i in range(10):
            print '=================================='
            print '=================================='
            print 'starting iteration', i
            for j in range(2):
                print 'Moving to location', locations[j]
                rvalue, dist = self.move_base_planner(*locations[j])
                print 'rvalue, dist', rvalue, dist
                rospy.sleep(3)
                base_poses[j].append(self.message)
                print j, 'Saved pose', base_poses[j][-1]
                
                self.message_lock.acquire()
                messages = copy.copy(self.messages)
                self.message_lock.release()

                print 'saving pickle'
                ut.save_pickle({'locations':locations, 'base_poses': base_poses, 'messages': messages}, 
                                'stats_04_poses_%s.pkl' % tstring)
                print 'saved!'
            print 'Finished iteration %d.' % i

if __name__ == '__main__':
    pe = PositionErrorExp()
    pe.run()

