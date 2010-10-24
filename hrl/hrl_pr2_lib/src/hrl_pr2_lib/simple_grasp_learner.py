#! /usr/bin/python
import numpy as np, math
import sys
from threading import RLock

import roslib; roslib.load_manifest('hrl_pr2_lib')
import rospy

import actionlib

from hrl_lib.util import save_pickle, load_pickle
from hrl_lib.transforms import rotX, rotY, rotZ
from hrl_pr2_lib.pr2_arms import PR2Arms
from hrl_pr2_lib.perception_monitor import ArmPerceptionMonitor

node_name = "simple_grasp_learner" 

def log(str):
    rospy.loginfo(node_name + ": " + str)

SETUP_POS = (0.62, 0.0, 0.035)
SETUP_POS_ANGS = [-0.1374468468502249, -0.34322445163698601, -1.2659495958735896, -1.0878155224042079, 4.895869827879376, -1.9892682489489215, -2.098935507952814]
ARM = 0 # right arm
NUM_X = 2 #7
NUM_Y = 2 #20
NUM_N = 2 #10
RECT = ((0.4, -0.77), (0.75, 0.23))
HOVER_Z = 0.035
MAX_JERK = 0.5
GRASP_DIST = 0.30 
GRASP_CONFIGS_FILE = "pickles//grasp_configs.pickle"
GRASP_DATA_FILE = "pickles//grasp_data.pickle"

def get_setup_pos_angs():
    arms = PR2Arms()
    arms.move_arm(ARM, SETUP_POS, rotY(np.pi / 2.), 4.)
    arms.wait_for_arm_completion(ARM)
    print arms.get_joint_angles(ARM)

def get_xy_list():
    grasp_xy_list = []
    for x in np.linspace(RECT[0][0], RECT[1][0], NUM_X):
        for y in np.linspace(RECT[0][1], RECT[1][1], NUM_Y):
            grasp_xy_list += [(x,y)]
    return grasp_xy_list

def save_grasp_configurations():
    arms = PR2Arms()

    grasp_xy_list = get_xy_list()
    grasp_configs = []
    setup = False

    for xy in grasp_xy_list:
        if not setup:
            # Move to setup position in the middle of the grasp space
            print "Setting up"
            arms.set_joint_angles(ARM, SETUP_POS_ANGS, 3.)
            arms.wait_for_arm_completion(ARM)
            rospy.sleep(0.5)
            setup = True

        if arms.can_move_arm(ARM, [xy[0], xy[1], HOVER_Z], rotY(np.pi / 2.)):
            print "Moving to pos (%1.2f, %1.2f)" % xy
            arms.move_arm(ARM, [xy[0], xy[1], HOVER_Z], rotY(np.pi / 2.), 3.)
            arms.wait_for_arm_completion(ARM)
            rospy.sleep(0.5)
            angs = arms.get_joint_angles(ARM)
            grasp_configs += [(xy, angs)]
            setup = False
        else:
            print "Can't move to to pos (%1.2f, %1.2f)" % xy
            grasp_configs += [(xy, None)]

    save_pickle(grasp_configs, GRASP_CONFIGS_FILE)
    print "Configurations:"
    print_configs(grasp_configs)

def print_configs(grasp_configs):
    for config in grasp_configs:
        if config[1] is not None:
            print "(%1.2f, %1.2f):" % config[0], ", ".join(["%1.2f" % x for x in config[1]])
        else:
            print "(%1.2f, %1.2f):" % config[0], "No config"

def collect_grasp_data(grasp_configs):
    arms = PR2Arms()
    apm = ArmPerceptionMonitor(ARM)

    grasp_data = []

    for config in grasp_configs:
        if not config[1] is None:
            # Do grasping num_n times
            for i in range(NUM_N):
                # Move to grasp position
                print "Moving to grasp position"
                arms.set_joint_angles(ARM, config[1], 3.)
                arms.wait_for_arm_completion(ARM)
                rospy.sleep(0.5)

                # start gathering data
                apm.start_training()
                # move arm down
                print "Moving arm down"
                arms.smooth_linear_arm_trajectory(ARM, GRASP_DIST, (0., 0., -1.), MAX_JERK)
                arms.wait_for_arm_completion(ARM)
                print "Finished moving arm"
                apm.stop_training()
                rospy.sleep(0.5)

            grasp_data += [(config[0], config[1], apm.datasets)]
            apm.clear_vars()
        else:
            grasp_data += [(config[0], None, None)]

    save_pickle(grasp_data, GRASP_DATA_FILE)

def display_grasp_data(grasp_data):
    import matplotlib.pyplot as plt
    plot_nums = [121, 122]
    colors = ['r', 'b', 'g', 'c']
    j = 0
    for pltnum in plot_nums:
        while grasp_data[j][1] is None:
            j += 1
        plt.subplot(pltnum)
        cnum = 0
        for stream in grasp_data[j][2]["accelerometer"]:
            print len(stream), len(stream[0])
            s_mag = [np.sqrt(x[1][0]**2 + x[1][1]**2 + x[1][2]**2) for x in stream]
            plt.plot(s_mag,colors[cnum])
            cnum += 1
            cnum %= len(colors)
        plt.title("%1.2f, %1.2f" % grasp_data[j][0])
        j += 1

    plt.show()

def main():
    rospy.init_node(node_name)
    # get_setup_pos_angs()
    # save_grasp_configurations()
    # print_configs(load_pickle(GRASP_CONFIGS_FILE))
    collect_grasp_data(load_pickle(GRASP_CONFIGS_FILE))
    display_grasp_data(load_pickle(GRASP_DATA_FILE))
    return 0

if __name__ == "__main__":
    sys.exit(main())
    
