
import sys
import numpy as np, math

import add_cylinder as ac
import online_collision_detection as ocd
import force_visualize_test as fvt

import roslib; roslib.load_manifest('arm_navigation_tutorials')
import rospy
from mapping_msgs.msg import CollisionObject
from visualization_msgs.msg import Marker

import hrl_lib.viz as hv

roslib.load_manifest('hrl_pr2_lib')
import hrl_pr2_lib.pr2_arms as pa


def contact_info_list_to_dict(cont_info_list):
    ci = cont_info_list[0]
    frm = ci.header.frame_id
#    print 'frame:', frm
    b1 = ci.contact_body_1
    b2 = ci.contact_body_2
    contact_dict = {}
    pts_list = []
    for ci in cont_info_list:
        if frm != ci.header.frame_id:
            rospy.logerr('initial frame_id: %s and current frame_id: %s'%(frm, ci.header.frame_id))

        b1 = ci.contact_body_1
        b2 = ci.contact_body_2
        two_bodies = b1 + '+' + b2
        if two_bodies not in contact_dict:
            contact_dict[two_bodies] = []
        contact_dict[two_bodies].append((ci.position.x, ci.position.y, ci.position.z))

    return contact_dict

def visualize_contact_dict(cd, marker_pub, fts):
    color_list = [(1.,0.,0.), (0.,1.,0.), (0.,0.,1.), (1.,1.,0.),
                  (1.,0.,1.), (0.,1.,1.), (0.5,1.,0.), (0.5,0.,1.),
                  (0.,0.5,1.) ]
    pts_list = []
    cs_list = []
    marker_list = []
    for i, k in enumerate(cd.keys()):
        pts = np.matrix(cd[k]).T
        c = color_list[i]
        cs = np.ones((4, pts.shape[1]))
        cs[0,:] = c[0]
        cs[1,:] = c[1]
        cs[2,:] = c[2]
        pts_list.append(pts)
        cs_list.append(cs)
        print '# of contact points:', pts.shape[1]
        mn = np.mean(pts, 1)

        f = fts.get_forces()
        m1, m2 = fvt.get_arrow_text_markers(mn, f, 'base_footprint',
                                            m_id = 2*i+1, duration=0.5)
        marker_pub.publish(m1)
        marker_pub.publish(m2)

    m = hv.list_marker(np.column_stack(pts_list),
                       np.column_stack(cs_list), (0.01, 0.01, 0.01),
                       'points', 'base_footprint', duration=1.0,
                       m_id=0)
    t_now = rospy.Time.now()
    m.header.stamp = t_now
    marker_pub.publish(m)

    for m in marker_list:
        m.header.stamp = rospy.Time.now()
        marker_pub.publish(m)


if __name__ == '__main__':
    rospy.init_node('pr2_skin_simulate')
    pub = rospy.Publisher('collision_object', CollisionObject)
    marker_pub = rospy.Publisher('/skin/viz_marker', Marker)

    fts = fvt.object_ft_sensors()
    fts.bias_fts()
    pr2_arms = pa.PR2Arms()
    r_arm, l_arm = 0, 1
    arm = r_arm

    raw_input('Touch the object and then hit ENTER.')

    ee_pos, ee_rot = pr2_arms.end_effector_pos(arm)
    print 'ee_pos:', ee_pos.flatten()
    print 'ee_pos.shape:', ee_pos.shape


    trans, quat = pr2_arms.tf_lstnr.lookupTransform('/base_footprint',
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
            contact_dict = contact_info_list_to_dict(res.contacts)
            print 'contact_dict.keys:', contact_dict.keys()
            visualize_contact_dict(contact_dict, marker_pub, fts)



