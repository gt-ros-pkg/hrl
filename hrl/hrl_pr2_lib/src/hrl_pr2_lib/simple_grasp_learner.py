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
from tf.transformations import *
from hrl_pr2_lib.pr2_arms import PR2Arms
from hrl_pr2_lib.perception_monitor import ArmPerceptionMonitor

# from pr2_gripper_reactive_approach.controller_manager import ControllerManager
from hrl_pr2_lib.hrl_controller_manager import HRLControllerManager as ControllerManager
from pr2_controllers_msgs.msg import JointTrajectoryGoal
import object_manipulator.convert_functions as cf
from object_manipulator.cluster_bounding_box_finder import ClusterBoundingBoxFinder
from tabletop_object_detector.srv import TabletopDetection
from tabletop_object_detector.msg import TabletopDetectionResult

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
NUM_X = 5#7 #4#7
NUM_Y = 5#9 #4#20
NUM_N = 3 #4
RECT = ((0.45, 0.10), (0.65, -0.10))#((0.4, 0.20), (0.7, -0.20))#((0.4, 0.35), (0.8, -0.35)) #((0.55, 0.10), (0.7, -0.10)) #((0.4, -0.77), (0.75, 0.23))
HOVER_Z = -0.10
MAX_JERK = 0.2
GRASP_DIST = 0.30 
GRASP_TIME = 2.0
STD_DEV = 3.5 #1.9
NOISE_DEV = 0.2
GRIPPER_POINT = np.array([0.23, 0.0, 0.0])
JOINTS_BIAS = [0., -.25, -1., 0., 0., 0.5, 0.]
BIAS_RADIUS = 0.012
DETECT_ERROR = -0.02
GRASP_CONFIGS_FILE = "pickles//grasp_configs.pickle"
GRASP_DATA_FILE = "pickles//grasp_data.pickle"
GRASP_MODELS_FILE = "pickles//grasp_models.pickle"
ZEROS_FILE = "pickles//current_zeros.pickle"
MONITOR_WINDOW = 200
PERCEPT_MON_LIST = ["accelerometer"]

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

def create_goal_pose(x, y, z, gripper_rot= 0.):
    quat1 = quaternion_about_axis(np.pi/2., (0, 1, 0))
    quat2 = quaternion_about_axis(gripper_rot, (0, 0, 1))
    quat = quaternion_multiply(quat2, quat1)
    point = [x, y, z]
    point = transform_in_frame(point, quat, -GRIPPER_POINT).tolist()
    pose = point + quat.tolist()
    goal_pose = cf.create_pose_stamped(pose, "torso_lift_link")
    goal_pose.header.stamp = rospy.Time.now()
    return goal_pose

def move_to_grasp_pos(cm, grasp_pose, block=True):
    grasp_pose.header.stamp = rospy.Time.now()
    cm.move_cartesian_ik(grasp_pose, collision_aware = False, blocking = block,
                      step_size = .005, pos_thres = .02, rot_thres = .1,
                      settling_time = rospy.Duration(1.0))
                      # joints_bias = JOINTS_BIAS, bias_radius = BIAS_RADIUS)

def downward_grasp(cm, goal_pose, block=True):
    goal_pose.header.stamp = rospy.Time.now()
    return cm.move_cartesian_ik(goal_pose, collision_aware = False, blocking = block,
                      step_size = .005, pos_thres = .02, rot_thres = .1,
                      settling_time = rospy.Duration(GRASP_TIME),
                      joints_bias = JOINTS_BIAS, bias_radius = BIAS_RADIUS)

def save_grasp_configurations():
    arms = PR2Arms()

    grasp_xy_list = get_xy_list()
    grasp_configs = []
    setup = False

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
            setup = False
        else:
            print "Can't move to to pos (%1.2f, %1.2f)" % xy
            # grasp_configs += [(xy, None)]

    save_pickle(grasp_configs, GRASP_CONFIGS_FILE)
    print "Configurations:"
    print_configs(grasp_configs)

def print_configs(grasp_configs):
    for config in grasp_configs:
        if config[1] is not None:
            print "(%1.2f, %1.2f):" % config[0], ", ".join(["%1.2f" % x for x in config[1]])
        else:
            print "(%1.2f, %1.2f):" % config[0], "No config"

def trim_bad_grasps(filename):
    grasp_configs = load_pickle(filename)
    print_configs(grasp_configs)
    new_configs = []
    for config in grasp_configs:
        if config[1] is not None:
            new_configs += [config]
    print_configs(new_configs)
    save_pickle(new_configs, filename)

def collect_grasp_data(grasp_configs, generate_models=False, use_config=True):
    cm = ControllerManager(armc)
    apm = ArmPerceptionMonitor(ARM)

    grasp_data = []

    print "Opening gripper"
    cm.command_gripper(0.08, -1.0, False)
    cm.wait_joint_trajectory_done()
    cm.gripper_action_client.wait_for_result(rospy.Duration(4.0))

    for config in grasp_configs:
        if not config[1] is None:
            zeros = None
            # Do grasping num_n times
            for i in range(NUM_N):
                # Move to grasp position
                print "Moving to grasp position (%1.2f, %1.2f)" % config[0]
                if use_config:
                    cm.command_joint(config[1])
                    cm.wait_joint_trajectory_done()
                else:
                    grasp_pose = create_goal_pose(config[0][0], config[0][1], HOVER_Z)
                    move_to_grasp_pos(cm, grasp_pose, block=False)
                rospy.sleep(0.5)
                if zeros is None:
                    # wait for a bit to get the zeros here
                    zeros = apm.get_zeros()

                goal_pose = create_goal_pose(config[0][0], config[0][1], 
                                              HOVER_Z - GRASP_DIST)
                # start gathering data
                apm.start_training()
                # move arm down
                print "Moving arm down"
                result = downward_grasp(cm, goal_pose, block=True)
                print result
                print "Finished moving arm"
                apm.stop_training()
                rospy.sleep(0.5)

            if result != "no solution" and generate_models:
                models = apm.generate_models()
            else:
                models = None
            if result != "no solution":
                grasp_data += [(config[0], config[1], apm.datasets, models, zeros)]
            else:
                print "Grasp Failed, not adding data"
            apm.clear_vars()
        # else:
            # grasp_data += [(config[0], None, None, None, None)]

    save_pickle(grasp_data, GRASP_DATA_FILE)

def load_data_and_generate(grasp_data):
    ret = []
    n = 0
    for grasp in grasp_data:
        apm = ArmPerceptionMonitor(ARM, percept_mon_list=PERCEPT_MON_LIST)
        apm.datasets = grasp[2]
        apm.generate_models()
        ret += [(grasp[0], grasp[1], grasp[2], apm.models, grasp[4])]
        n += 1
        print "Generated %d models" % (n)
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

def save_current_zeros(filename=ZEROS_FILE):
    apm = ArmPerceptionMonitor(ARM, percept_mon_list=PERCEPT_MON_LIST)
    zeros = apm.get_zeros()
    print "Zeros:", zeros
    save_pickle(zeros, filename)

def load_current_zeros(filename=ZEROS_FILE):
    return load_pickle(filename)

def open_gripper(cm=None):
    if cm is None:
        cm = ControllerManager(armc)
    cm.command_gripper(0.08, -1.0, False)
    cm.gripper_action_client.wait_for_result(rospy.Duration(4.0))

def perform_grasp(grasp_data, x, y, z=None, gripper_rot = np.pi / 2., is_place=False, 
                  cm=None, apm=None):
    if cm is None:
        cm = ControllerManager(armc)
    if apm is None:
        apm = ArmPerceptionMonitor(ARM, percept_mon_list=PERCEPT_MON_LIST)

    def dist(g):
        return np.sqrt((g[0][0] - x) ** 2 + (g[0][1] - y) ** 2)
    grasp = min(grasp_data, key=dist)
    print "Distance to grasp model:", dist(grasp)

    apm.datasets = grasp[2]
    apm.models = grasp[3]
    apm.model_zeros = grasp[4]
    print "Model zeros:", apm.model_zeros

    print "Beginning grasp"

    # Move to grasp position
    print "Moving to initial grasp position"
    cm.command_joint(grasp[1])
    cm.wait_joint_trajectory_done()

    print "Moving to final grasp position"
    grasp_pose = create_goal_pose(x, y, HOVER_Z, gripper_rot)
    move_to_grasp_pos(cm, grasp_pose, block=False)
    if not is_place:
        # open gripper
        print "Opening gripper"
        cm.command_gripper(0.08, -1.0, False)
        cm.gripper_action_client.wait_for_result(rospy.Duration(4.0))
    cm.wait_joint_trajectory_done()
    rospy.sleep(0.5)

    zeros = apm.get_zeros(1.0)
    print "Current zeros:", zeros

    goal_pose = create_goal_pose(x, y, HOVER_Z - GRASP_DIST, gripper_rot)

    # start gathering data
    def print_collision():
        print "Collision!"
        while not cm.check_joint_trajectory_done():
            cm.joint_action_client.cancel_all_goals()
            rospy.sleep(0.01)
    apm.begin_monitoring(contingency=print_collision, window_size=MONITOR_WINDOW,
                         current_zeros=zeros, std_dev_default=STD_DEV, 
                         noise_dev_default=NOISE_DEV)

    # move arm down
    print "Moving arm down"
    downward_grasp(cm, goal_pose, block = False)
    if z is None:
        cm.wait_joint_trajectory_done()
    else:
        while not cm.check_joint_trajectory_done():
            wrist_pose = cm.get_current_wrist_pose_stamped("torso_lift_link")
            p = wrist_pose.pose.position
            o = wrist_pose.pose.orientation
            affector_pos = transform_in_frame([p.x, p.y, p.z],
                                              [o.x, o.y, o.z, o.w], GRIPPER_POINT)
            if affector_pos[2] <= z:
                print "Reached z position"
                print "Affector position:", affector_pos
                while not cm.check_joint_trajectory_done():
                    cm.joint_action_client.cancel_all_goals()
                    rospy.sleep(0.01)
                break
            rospy.sleep(0.01)

    print "Finished moving arm"

    avg_list = apm.end_monitoring()
    rospy.sleep(0.5)

    if not is_place:
        print "Closing gripper"
        cm.command_gripper(0.0, 30.0, True)
        print "Gripper closed"
    else:
        print "Opening gripper"
        cm.command_gripper(0.08, -1.0, True)
        print "Gripper opened"

    print "Moving back to grasp position"
    move_to_grasp_pos(cm, grasp_pose, block=True)
    print "Grasp complete!"
    
    return avg_list

# def perform_template_grasp(grasp):
#     cm = ControllerManager(armc)
#     apm = ArmPerceptionMonitor(ARM, percept_mon_list=["accelerometer"])
# 
#     apm.datasets = grasp[2]
#     apm.models = grasp[3]
# 
#     # Move to grasp position
#     print "Moving to grasp position"
#     cm.command_joint(grasp[1])
#     cm.wait_joint_trajectory_done()
#     rospy.sleep(0.5)
# 
#     goal_pose = create_goal_pose(grasp[0][0], grasp[0][1], HOVER_Z - GRASP_DIST)
# 
#     # start gathering data
#     def print_collision():
#         print "Collision!"
#         cm.freeze_arm()
#     apm.begin_monitoring(contingency=print_collision, window_size=MONITOR_WINDOW)
# 
#     # move arm down
#     print "Moving arm down"
#     downward_grasp(cm, goal_pose, block = False)
#     cm.wait_joint_trajectory_done()
#     print "Finished moving arm"
# 
#     avg_list = apm.end_monitoring()
#     rospy.sleep(0.5)
# 
#     return avg_list

def display_grasp_data(grasp_data, percept="accelerometer", indicies=range(3), 
                       std_dev=STD_DEV, noise_dev_add=NOISE_DEV, monitor_data=[], 
                       plot_data=False, rows=2, colors=None):
    import matplotlib.pyplot as plt
    if colors is None:
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
        zip_monitors = []
        for d in monitor_data:
            zip_monitors += [zip(*d[percept])]
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
            for zip_monitor in zip_monitors:
                len_diff = len(graph_means[k]) - len(zip_monitor[indicies[k]])
                add_vals = [zip_monitor[indicies[k]][0]] * MONITOR_WINDOW
                # add_vals = [zip_monitor[indicies[k]][0]] * len_diff
                print len_diff
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

def test_num_samples(percept="accelerometer"):
    # This function proves that adding more than 3-4 grasps per location is unnecessary
    # Grasping trajectories are very repeatable given a specific grasping location.
    # The issue comes from more from shifting of the actual trajectory from expected
    # given a grasping location just a few cm off.  The models can handle the trajectory
    # so long as it is not much more than 5cm away from the training model.

    grasp_data = load_pickle(GRASP_DATA_FILE)[0:1]
    data_list = grasp_data[0][2][percept]
    for i in [3, 6, 9]:
        apm = ArmPerceptionMonitor(ARM, percept_mon_list=PERCEPT_MON_LIST)
        grasp_data[0][2][percept] = data_list[0:i+1]
        apm.datasets = grasp_data[0][2]
        apm.generate_models()
        grasp_data[0][3][percept] = apm.models[percept]
        if i != 3:
            colors = ['m', 'y', 'k', 'r']
        else:
            colors = None
        display_grasp_data(grasp_data, percept, rows=1)

def detect_tabletop_objects():
    cbbf = ClusterBoundingBoxFinder()
    object_detector = rospy.ServiceProxy("/object_detection", TabletopDetection)
    detects = object_detector(True, False).detection
    object_detector.close()
    if detects.result != 4:
        print "Detection failed (err %d)" % (detects.result)
    table_z = detects.table.pose.pose.position.z
    objects = []
    for cluster in detects.clusters:
        (object_points, 
         object_bounding_box_dims, 
         object_bounding_box, 
         object_to_base_link_frame, 
         object_to_cluster_frame) = cbbf.find_object_frame_and_bounding_box(cluster)
        # print "bounding box:", object_bounding_box
        (object_pos, object_quat) = cf.mat_to_pos_and_quat(object_to_cluster_frame)
        angs = euler_from_quaternion(object_quat)
        print "angs:", angs
        # position is half of height
        object_pos[2] = table_z + object_bounding_box[1][2] / 2. + DETECT_ERROR
        print "pos:", object_pos
        print "table_z:", table_z
        objects += [[object_pos, angs[2]]]
    return objects

def grasp_closest_object(grasp_data, x, y, cm=None, apm=None):
    objects = detect_tabletop_objects()
    if len(objects) > 0:
        def dist(o):
            return (o[0][0] - x) ** 2 + (o[0][1] - y) ** 2
        obj = min(objects, key=dist)
        return perform_grasp(grasp_data, obj[0][0], obj[0][1], #z=obj[0][2], 
                             gripper_rot=obj[1], cm=cm, apm=apm)
    else:
        print "No objects near point"
        return None

def trim_test_data():
    grasp_data = load_pickle(GRASP_DATA_FILE)
    new_data = []
    for grasp in grasp_data:
        new_data += [(grasp

def main():
    rospy.init_node(node_name)
    # rospy.on_shutdown(die)

    cm = ControllerManager(armc)
    apm = ArmPerceptionMonitor(ARM, percept_mon_list=PERCEPT_MON_LIST)
    # print detect_tabletop_objects()
    # return 0

    # get_setup_pos_angs()
    # print_configs(load_pickle(GRASP_CONFIGS_FILE))

    # save_grasp_configurations()
    # return 0
    # import random
    # data = load_pickle(GRASP_CONFIGS_FILE)
    # random.shuffle(data)
    # collect_grasp_data(data)
    # return 0
    # data = load_pickle(GRASP_CONFIGS_FILE)
    # new_data = []
    # xys = get_xy_list()
    # for grasp in data:
    #     for xy in xys:
    #         if np.allclose(xy, grasp[0]):
    #             new_data += [grasp]

    # print "length:", len(new_data)
    # collect_grasp_data(new_data)
    # collect_grasp_data(load_pickle(GRASP_CONFIGS_FILE))
    # return 0
    # trim_bad_grasps("gcb2.pickle")
    # return 0
    # save_current_zeros()
    # return 0
    # test_num_samples()
    # return 0

    # print "Generating data"
    # grasp_data = load_data_and_generate(load_pickle(GRASP_DATA_FILE))
    # save_pickle(grasp_data, GRASP_DATA_FILE)
    # print "Done generating data"
    # return 0

    grasp_data = load_pickle(GRASP_DATA_FILE)
    
    monitor_data = []

    monitor_data += [grasp_closest_object(grasp_data, 0.48, 0.07, cm=cm, apm=apm)]
    # grasp_data = load_pickle(GRASP_DATA_FILE)
    # print grasp_data[0][0]
    # monitor_data = perform_template_grasp(grasp_data[0])
    # display_grasp_data(grasp_data, "accelerometer", monitor_data=monitor_data, rows=1)
    # display_grasp_data(grasp_data, "r_finger_periph_pressure", rows=1)
    # monitor_data = perform_grasp(grasp_data, .58, .14, gripper_rot = 0.)
    # monitor_data += [perform_grasp(grasp_data, .55, .10, gripper_rot = 0.)]
    # monitor_data += [perform_grasp(grasp_data, .55, .15, gripper_rot = 0., is_place=True)]
    # monitor_data += [perform_grasp(grasp_data, .56, .10, gripper_rot = 0.)]
    # monitor_data += [perform_grasp(grasp_data, .57, .10, gripper_rot = 0.)]
    # monitor_data += [perform_grasp(grasp_data, .58, .10, gripper_rot = 0.)]

    # for i in range(2):
    #     monitor_data += [perform_grasp(grasp_data, .55, .10, gripper_rot = 0., cm=cm, apm=apm)]
    #     monitor_data += [perform_grasp(grasp_data, .55, .15, gripper_rot = 0., is_place=True, cm=cm, apm=apm)]
    #     monitor_data += [perform_grasp(grasp_data, .55, .15, gripper_rot = 0., cm=cm, apm=apm)]
    #     monitor_data += [perform_grasp(grasp_data, .55, .10, gripper_rot = 0., is_place=True, cm=cm, apm=apm)]
    # monitor_data += [perform_grasp(grasp_data, .55, .10, z=HOVER_Z-0.14, gripper_rot = 0., is_place=False, cm=cm, apm=apm)]
    display_grasp_data(grasp_data, "accelerometer", monitor_data=monitor_data, rows=1)
    raw_input()
    open_gripper(cm=cm)
    # test_monitor(load_pickle(GRASP_DATA_FILE))
    return 0

if __name__ == "__main__":
    sys.exit(main())
    
