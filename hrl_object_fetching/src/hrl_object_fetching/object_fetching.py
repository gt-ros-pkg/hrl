#! /usr/bin/python
import random
import sys

import roslib; roslib.load_manifest("hrl_object_fetching")
import rospy
import actionlib

from pr2_overhead_grasping.msg import *
#from pr2_playpen.srv import *
from hrl_lib.keyboard_input import KeyboardInput
from pr2_overhead_grasping.helpers import log
from tabletop_object_detector.srv import TabletopSegmentation
from tabletop_object_detector.msg import TabletopDetectionResult

##
# Uses the object_detection service detect objects on the table. Returns a list
# of pairs [ [ x, y, z], rot ] which represent the centers and z orientation of
# each object detected on the table.
def detect_tabletop_objects():
    DETECT_ERROR = 0.
    #cbbf = ClusterBoundingBoxFinder(tf_listener=self.oger.cm.tf_listener)
    object_detector = rospy.ServiceProxy("/tabletop_segmentation", TabletopSegmentation)
    table = object_detector().table
    object_detector.close()
    return table
    #if detects.result != 4:
    #    err("Detection failed (err %d)" % (detects.result))
    #    return []
    #table_z = detects.table.pose.pose.position.z
    #objects = []
    #for cluster in detects.clusters:
    #    (object_points, 
    #     object_bounding_box_dims, 
    #     object_bounding_box, 
    #     object_to_base_link_frame, 
    #     object_to_cluster_frame) = cbbf.find_object_frame_and_bounding_box(cluster)
    #    # log("bounding box:", object_bounding_box)
    #    (object_pos, object_quat) = cf.mat_to_pos_and_quat(object_to_cluster_frame)
    #    angs = euler_from_quaternion(object_quat)
    #    log("angs:", angs)
    #    # position is half of height
    #    object_pos[2] = table_z + object_bounding_box[1][2] / 2. + DETECT_ERROR
    #    log("pos:", object_pos)
    #    log("table_z:", table_z)
    #    objects += [[object_pos, angs[2]]]
    #return objects
            

if __name__ == '__main__':
    args = sys.argv
    rospy.init_node('object_fetching')
    print detect_tabletop_objects()
#    ki = KeyboardInput()
#    grasp_client = actionlib.SimpleActionClient('overhead_grasp', OverheadGraspAction)
#    grasp_client.wait_for_server()
#    grasp_setup_client = actionlib.SimpleActionClient('overhead_grasp_setup', OverheadGraspSetupAction)
#    grasp_setup_client.wait_for_server()
#    print "grasp_client ready"
#    if args[1] == 'reg_test':
#        results_dict = {}
#        ki.kbhit()
#        while not rospy.is_shutdown():
#            goal = OverheadGraspSetupGoal()
#            grasp_setup_client.send_goal(goal)
#            #grasp_setup_client.wait_for_result()
#            rospy.sleep(1.0)
#
#            print "Grasp started"
#            goal = OverheadGraspGoal()
#            goal.is_grasp = True
#            goal.grasp_type=OverheadGraspGoal.VISION_GRASP
#            goal.x = 0.5 + random.uniform(-0.1, 0.1)
#            goal.y = 0.0 + random.uniform(-0.1, 0.1)
#            grasp_client.send_goal(goal)
#            grasp_client.wait_for_result()
#            result = grasp_client.get_result()
#            goal = OverheadGraspGoal()
#            if result.grasp_result == "Table collision" or True:
#                goal.is_grasp = False
#                goal.grasp_type=OverheadGraspGoal.MANUAL_GRASP
#                goal.x = 0.5 + random.uniform(-0.1, 0.1)
#                goal.y = 0.0 + random.uniform(-0.1, 0.1)
#                grasp_client.send_goal(goal)
#                grasp_client.wait_for_result()
#                result = grasp_client.get_result()
#            if result.grasp_result not in results_dict:
#                results_dict[result.grasp_result] = 0
#            results_dict[result.grasp_result] += 1
#
#            print results_dict
#            try:
#                print float(results_dict["No collision"]) / results_dict["Regular collision"]
#            except:
#                pass
#    
#    elif args[1] == 'playpen_demo':
#        rospy.wait_for_service('conveyor')
#        conveyor = rospy.ServiceProxy('conveyor', Conveyor)
#        print "conveyor ready"
#        rospy.wait_for_service('playpen')
#        playpen = rospy.ServiceProxy('playpen', Playpen)
#        print "playpen ready"
#        print "waiting for setup"
#        rospy.sleep(20.0)
#        while not rospy.is_shutdown():
#            conveyor(0.10 * 2.)
#
#            print "Grasp started"
#            goal = OverheadGraspGoal()
#            goal.is_grasp = True
#            goal.grasp_type=OverheadGraspGoal.VISION_GRASP
#            goal.x = 0.5
#            goal.y = 0.0
#            grasp_client.send_goal(goal)
#            grasp_client.wait_for_result()
#            result = grasp_client.get_result()
#            goal = OverheadGraspGoal()
#            goal.is_grasp = False
#            goal.grasp_type=OverheadGraspGoal.MANUAL_GRASP
#            goal.x = 0.5 + random.uniform(-0.1, 0.1)
#            goal.y = 0.0 + random.uniform(-0.1, 0.1)
#            grasp_client.send_goal(goal)
#            grasp_client.wait_for_result()
#            result = grasp_client.get_result()
#
#            # do playpen stuff
#
#            log("Opening trap door")
#            playpen(1)
#            rospy.sleep(0.2)
#            log("Closing trap door")
#            playpen(0)
#            rospy.sleep(0.2)



