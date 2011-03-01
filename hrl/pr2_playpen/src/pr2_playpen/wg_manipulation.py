import roslib
roslib.load_manifest('pr2_playpen')
import rospy
from pr2_playpen.pick_and_place_manager import *
#from pr2_pick_and_place_demos.pick_and_place_manager import *
from object_manipulator.convert_functions import *
from pr2_playpen.srv import Playpen
from pr2_playpen.srv import Conveyor
import math
import numpy as np

class SimplePickAndPlaceExample():

    def __init__(self):

        rospy.loginfo("initializing pick and place manager")
        self.papm = PickAndPlaceManager()
        rospy.loginfo("finished initializing pick and place manager")
        rospy.wait_for_service('playpen')
        rospy.wait_for_service('conveyor')
        self.playpen = rospy.ServiceProxy('playpen', Playpen)
        self.conveyor = rospy.ServiceProxy('conveyor', Conveyor)
        self.objects_dist = [.135, .26-.135, .405-.26, .545-.405, 
                             .70-.545, .865-.70, .995-.865, 1.24-.995]
        self.tries = 0
        self.successes = 0
    #pick up the nearest object to PointStamped target_point with whicharm 
    #(0=right, 1=left)
    def pick_up_object_near_point(self, target_point, whicharm):

        rospy.loginfo("moving the arms to the side")
        self.papm.move_arm_to_side(0)  #right arm
        self.papm.move_arm_to_side(1)  #left arm


#############once is it positioned, we don't want to move the head at all !!!#############
#        rospy.loginfo("pointing the head at the target point")
#        self.papm.point_head(get_xyz(target_point.point),
#                             target_point.header.frame_id)
#########################################################################################        


        rospy.loginfo("detecting the table and objects")
        self.papm.call_tabletop_detection(take_static_collision_map = 1,
                             update_table = 1, clear_attached_objects = 1)

        rospy.loginfo("picking up the nearest object to the target point")
        success = self.papm.pick_up_object_near_point(target_point,
                                                      whicharm)

        if success:
            rospy.loginfo("pick-up was successful!  Moving arm to side")
            self.papm.move_arm_to_side(whicharm)
        else:
            rospy.loginfo("pick-up failed.")

        return success


    #place the object held in whicharm (0=right, 1=left) down in the 
    #place rectangle defined by place_rect_dims (x,y) 
    #and place_rect_center (PoseStamped)
    def place_object(self, whicharm, place_rect_dims, place_rect_center):

        self.papm.set_place_area(place_rect_center, place_rect_dims)

        rospy.loginfo("putting down the object in the %s gripper"\
                      %self.papm.arm_dict[whicharm])
        success = self.papm.put_down_object(whicharm,
                      max_place_tries = 5,
                      use_place_override = 1)

        if success:
            rospy.loginfo("place returned success")
        else:
            rospy.loginfo("place returned failure")

        return success


if __name__ == "__main__":
    rospy.init_node('simple_pick_and_place_example')
    sppe = SimplePickAndPlaceExample()


    #adjust for your table 
    table_height = 0.529

############here is where we put the loop and service calls to know if successful or not...., when done write success rate/statistics to pkl file
############and call service to advance to next object, etc.

    #.5 m in front of robot, centered
    target_point_xyz = [.625, 0, table_height]   #this is currently an approximation/hack should us ar tag
    target_point = create_point_stamped(target_point_xyz, 'base_link')
    arm = 0

    for i in xrange(len(sppe.objects_dist)):
        sppe.playpen(0)
        sppe.conveyor(sppe.objects_dist[i])

        while sppe.tries<2:
            print "arm is ", arm
#            print "target _point is ", target_point.x
            success = sppe.pick_up_object_near_point(target_point, arm)   #right arm

            if success:
                sppe.successes=sppe.successes + 1
                #square of size 30 cm by 30 cm
                place_rect_dims = [.1, .1]

                
                #.5 m in front of robot, to the right
                radius = np.random.uniform(0,0.20, 1)[0]
                angle = np.random.uniform(0, 2*math.pi, 1)[0]
                center_xyz = [.625+math.cos(angle)*radius, math.sin(angle)*radius, table_height+.2]

                #aligned with axes of frame_id
                center_quat = [0,0,0,1]
                place_rect_center = create_pose_stamped(center_xyz+center_quat,
                                                        'base_link')

                sppe.place_object(arm, place_rect_dims, place_rect_center)

            arm = arm.__xor__(1)
            sppe.tries = sppe.tries+1

        sppe.playpen(1)
        sppe.successes = 0
        sppe.tries = 0

#    sppe.playpen(0)
