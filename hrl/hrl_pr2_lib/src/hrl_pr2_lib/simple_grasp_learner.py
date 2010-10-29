#! /usr/bin/python
import numpy as np, math
import sys
from threading import RLock

import roslib; roslib.load_manifest('hrl_pr2_lib')
import rospy

import actionlib

from geometry_msgs.msg import  Point, Pose, Quaternion, PoseStamped

from hrl_lib.util import save_pickle, load_pickle
from hrl_lib.transforms import rotX, rotY, rotZ
from tf.transformations import quaternion_about_axis, quaternion_matrix
from hrl_pr2_lib.pr2_arms import PR2Arms
from hrl_pr2_lib.perception_monitor import ArmPerceptionMonitor

from pr2_gripper_reactive_approach.controller_manager import ControllerManager
import object_manipulator.convert_functions as cf

node_name = "simple_grasp_learner" 

def log(str):
    rospy.loginfo(node_name + ": " + str)

SETUP_POS = (0.62, 0.0, 0.035)
SETUP_POS_ANGS = [-0.6260155429349421, -0.53466276262236689, -1.9803303473514324, -1.1593322538276705, -0.89803655400181404, -1.4467120153069799, -2.728422563953746]
ARM = 0 # right arm
if ARM == 0:
    armc = 'r'
else:
    armc = 'l'
NUM_X = 4#7
NUM_Y = 4#20
NUM_N = 10
RECT = ((0.55, 0.10), (0.7, -0.10)) #((0.4, -0.77), (0.75, 0.23))
HOVER_Z = -0.10
MAX_JERK = 0.2
GRASP_DIST = 0.30 
GRIPPER_POINT = np.array([0.23, 0.0, 0.0])
GRASP_CONFIGS_FILE = "pickles//grasp_configs.pickle"
GRASP_DATA_FILE = "pickles//grasp_data.pickle"
MONITOR_WINDOW = 70

##
# Transforms the given position by the offset position in the given quaternion
# rotation frame
#
# @param pos the current positions
# @param quat quaternion representing the rotation of the frame
# @param off_point offset to move the position inside the quat's frame
# @return the new position as a matrix column
def transform_in_frame(pos, quat, off_point):
    invquatmat = np.mat(quaternion_matrix(quat))
    invquatmat[0:3,3] = np.mat(pos).T
    trans = np.matrix([off_point[0],off_point[1],off_point[2],1.]).T
    transpos = invquatmat * trans
    return transpos.T.A[0,0:3]

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
#   setup = False

    for xy in grasp_xy_list:
#       if not setup:
#           # Move to setup position in the middle of the grasp space
#           print "Setting up"
#           arms.set_joint_angles(ARM, SETUP_POS_ANGS, 3.)
#           arms.wait_for_arm_completion(ARM)
#           rospy.sleep(0.5)
#           setup = True

        if arms.can_move_arm(ARM, [xy[0], xy[1], HOVER_Z], rotY(np.pi / 2.)):
            print "Moving to pos (%1.2f, %1.2f)" % xy
            arms.grasp_biased_move_arm(ARM, [xy[0], xy[1], HOVER_Z], rotY(np.pi / 2.), 3.)
            arms.wait_for_arm_completion(ARM)
            rospy.sleep(0.5)
            angs = arms.get_joint_angles(ARM)
            grasp_configs += [(xy, angs)]
#           setup = False
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
    cm = ControllerManager(armc)
    apm = ArmPerceptionMonitor(ARM)

    grasp_data = []
    quat = quaternion_about_axis(np.pi / 2., (0, 1, 0))

    for config in grasp_configs:
        if not config[1] is None:
            # Do grasping num_n times
            for i in range(NUM_N):
                # Move to grasp position
                print "Moving to grasp position"
                cm.command_joint(config[1])
                cm.wait_joint_trajectory_done()
                rospy.sleep(0.5)

                point = [config[0][0], config[0][1], HOVER_Z - GRASP_DIST]
                point = transform_in_frame(point, quat, -GRIPPER_POINT).tolist()
                pose = point + quat.tolist()
                goal_pose = cf.create_pose_stamped(pose, "torso_lift_link")
                goal_pose.header.stamp = rospy.Time.now()

                # start gathering data
                apm.start_training()
                # move arm down
                print "Moving arm down"
                cm.move_cartesian_ik(goal_pose, collision_aware = False, blocking = True,
                                  step_size = .005, pos_thres = .02, rot_thres = .1,
                                  settling_time = rospy.Duration(4.0))
                print "Finished moving arm"
                apm.stop_training()
                rospy.sleep(0.5)

            models = apm.generate_models()
            grasp_data += [(config[0], config[1], apm.datasets, models)]
            apm.clear_vars()
        else:
            grasp_data += [(config[0], None, None, None)]

    save_pickle(grasp_data, GRASP_DATA_FILE)

def load_data_and_generate(grasp_data):
    ret = []
    for grasp in grasp_data:
        apm = ArmPerceptionMonitor(ARM)
        apm.datasets = grasp[2]
        apm.generate_models()
        ret += [(grasp[0], grasp[1], grasp[2], apm.models)]
    return ret

def test_monitor(grasp_data):
    apm = ArmPerceptionMonitor(ARM)
    j = 0
    while grasp_data[j][1] is None:
        j += 1
    apm.models = grasp_data[j][3]
    def print_warn():
        print "Warning! Outside model"
    apm.begin_monitoring(contingency=print_warn)
    raw_input()
    apm.end_monitoring()

def perform_grasp(grasp):
    cm = ControllerManager(armc)
    apm = ArmPerceptionMonitor(ARM)

    apm.datasets = grasp[2]
    apm.models = grasp[3]
    quat = quaternion_about_axis(np.pi / 2., (0, 1, 0))

    # Move to grasp position
    print "Moving to grasp position"
    cm.command_joint(grasp[1])
    cm.wait_joint_trajectory_done()
    rospy.sleep(0.5)

    point = [grasp[0][0], grasp[0][1], HOVER_Z - GRASP_DIST]
    point = transform_in_frame(point, quat, -GRIPPER_POINT).tolist()
    pose = point + quat.tolist()
    goal_pose = cf.create_pose_stamped(pose, "torso_lift_link")
    goal_pose.header.stamp = rospy.Time.now()

    # start gathering data
    def print_collision():
        print "Collision!"
        cm.stop_controllers(stop_joint = 1)
    apm.begin_monitoring(contingency=print_collision, window_size=MONITOR_WINDOW)
    # move arm down
    print "Moving arm down"
    cm.move_cartesian_ik(goal_pose, collision_aware = False, blocking = True,
                      step_size = .005, pos_thres = .02, rot_thres = .1,
                      settling_time = rospy.Duration(4.0))
    print "Finished moving arm"
    avg_list = apm.end_monitoring()
    rospy.sleep(0.5)

    return avg_list

def display_grasp_data(grasp_data, percept="accelerometer", indicies=range(3), std_dev=2.0,
                       noise_dev_add=0.25, monitor_data=None, plot_data=False, rows=2):
    import matplotlib.pyplot as plt
    colors = ['r', 'b', 'g', 'c']
    j = 0
    cols = len(indicies)
    for i in range(rows):
        while grasp_data[j][1] is None:
            j += 1
        models = grasp_data[j][3]
        means = models[percept]["mean"]
        vars = models[percept]["variance"]
        signals = models[percept]["smoothed_signals"]
        noise_var = models[percept]["noise_variance"]
        noise_dev = np.sqrt(noise_var)
        print "noise variance", noise_var

        zip_means = np.array(zip(*means))
        zip_vars = np.array(zip(*vars))
        zip_signals = [ zip(*sig) for sig in signals ]
        # print "monitor_data", monitor_data
        if monitor_data is not None:
            zip_monitor = zip(*monitor_data[percept])
        else:
            zip_monitor = None
        graph_means, graph_devs, mmax, mmin = [], [], [], []
        for w in indicies:
            graph_means += [zip_means[w]]
            graph_devs += [np.sqrt(zip_vars[w])]

            mmax += [graph_means[w] + graph_devs[w] * std_dev + noise_dev[w] * noise_dev_add]
            mmin += [graph_means[w] - graph_devs[w] * std_dev - noise_dev[w] * noise_dev_add]

        for k in range(len(indicies)):
            plt.subplot(rows, cols, i*cols + 1 + k)
            cnum = 0
            for stream in grasp_data[j][2][percept]:
                # s_mag = [np.sqrt(x[1][0]**2 + x[1][1]**2 + x[1][2]**2) for x in stream]
                # plt.plot(s_mag,colors[cnum])
                if plot_data:
                    plt.plot([x[1][k] for x in stream],colors[cnum])
                    cnum += 1
                    cnum %= len(colors)
            for sig in zip_signals:
                plt.plot(sig[indicies[k]], colors[2])
            if zip_monitor is not None:
                # len_diff = len(graph_means[k]) - len(zip_monitor[indicies[k]])
                add_vals = [zip_monitor[indicies[k]][0]] * MONITOR_WINDOW
                g = add_vals + list(zip_monitor[indicies[k]])
                plt.plot(g,colors[3])
            plt.plot(graph_means[k], colors[0])
            plt.plot(mmax[k], colors[1])
            plt.plot(mmin[k], colors[1])
            plt.title("%1.2f, %1.2f: coord %d" % (grasp_data[j][0][0], grasp_data[j][0][1], k))
        j += 1

    plt.show()

def die():
    sys.exit()

def main():
    rospy.init_node(node_name)
    rospy.on_shutdown(die)
    # get_setup_pos_angs()
    # save_grasp_configurations()
    # print_configs(load_pickle(GRASP_CONFIGS_FILE))
    # collect_grasp_data(load_pickle(GRASP_CONFIGS_FILE)[0:1])
    grasp_data = load_pickle(GRASP_DATA_FILE)
    # grasp_data = load_data_and_generate(load_pickle(GRASP_DATA_FILE))
    # monitor_data = perform_grasp(grasp_data[0])
    display_grasp_data(grasp_data, "accelerometer", monitor_data=monitor_data, rows=1)
    # display_grasp_data(grasp_data, "r_finger_periph_pressure", rows=1)
    raw_input()
    # test_monitor(load_pickle(GRASP_DATA_FILE))
    return 0

if __name__ == "__main__":
    sys.exit(main())
    
