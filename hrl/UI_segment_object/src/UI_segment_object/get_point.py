
import roslib; roslib.load_manifest('UI_segment_object')

import rospy
from UI_segment_object.srv import GetPt
from UI_segment_object.srv import None_Bool

if __name__ == '__main__':
    print 'hello world'
    rospy.init_node('point_and_click_client')

    reset_srv_name = 'UI_reset'
    srv_name = 'get_3D_pt'

    rospy.loginfo('waiting for service: %s'%reset_srv_name)
    rospy.wait_for_service(reset_srv_name)
    rospy.loginfo('waiting for service: %s'%srv_name)
    rospy.wait_for_service(srv_name)
    rospy.loginfo('Done')

    reset_ui = rospy.ServiceProxy(reset_srv_name, None_Bool)
    get_3d_point = rospy.ServiceProxy(srv_name, GetPt)

    print 'Reset result:', reset_ui()
    resp = get_3d_point()
    print 'resp:', resp.pt.x, resp.pt.y, resp.pt.z



