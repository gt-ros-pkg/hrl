
import sys
import numpy as np, math

import add_cylinder as ac
import online_collision_detection as ocd

import roslib; roslib.load_manifest('arm_navigation_tutorials')
import rospy
from mapping_msgs.msg import CollisionObject

roslib.load_manifest('hrl_pr2_lib')
import hrl_pr2_lib.pr2_arms as pa



def contact_info_list_to_np(cont_info_list):
    ci = cont_info_list[0]
    frm = ci.header.frame_id
    b1 = ci.contact_body_1
    b2 = ci.contact_body_2
    pts_list = []
    for ci in cont_info_list:
        if frm != ci.header.frame_id:
            rospy.logerr('initial frame_id: %s and current frame_id: %s'%(frm, ci.header.frame_id))
        if ci.contact_body_1 != b1:
            rospy.logerr('initial body1: %s and current body1: %s'%(b1, ci.contact_body_1))
        if ci.contact_body_2 != b2:
            rospy.logerr('initial body2: %s and current body2: %s'%(b2, ci.contact_body_2))

        pts_list.append((ci.position.x, ci.position.y, ci.position.z))
    pts = np.matrix(pts_list).T
    return pts




if __name__ == '__main__':
    rospy.init_node('pr2_skin_simulate')
    pub = rospy.Publisher('collision_object', CollisionObject)

    pr2_arms = pa.PR2Arms()
    r_arm, l_arm = 0, 1
    arm = r_arm

    raw_input('Touch the object and then hit ENTER.')

    ee_pos, ee_rot = pr2_arms.end_effector_pos(arm)
    print 'ee_pos:', ee_pos.flatten()
    print 'ee_pos.shape:', ee_pos.shape


    trans, quat = pr2_arms.tf_lstnr.lookupTransform('/base_link',
                             '/torso_lift_link', rospy.Time(0))
    height = ee_pos[2] + trans[2]
    ee_pos[2] = -trans[2]
    ac.add_cylinder('pole', ee_pos, 0.02, height, '/torso_lift_link', pub)

    rospy.loginfo('Now starting the loop where I get contact locations.')
    col_det = ocd.online_collision_detector()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        res = col_det.check_validity(pr2_arms, arm)
        if res.error_code.val ==  res.error_code.SUCCESS:
            rospy.loginfo('No contact')
        else:
            contact_pts = contact_info_list_to_np(res.contacts)
            print 'contact_pts:'
            print contact_pts



