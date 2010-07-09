
import roslib; roslib.load_manifest('modeling_forces')
import rospy

from geometry_msgs.msg import Point32
from articulation_msgs.msg import ModelMsg
from articulation_msgs.msg import TrackMsg
from geometry_msgs.msg import Pose, Point, Quaternion

from threading import RLock
import numpy as np
import time

def model_cb(data, kin_dict):
    print 'model_cb called'
    if data.name == 'rotational':
        for p in data.params:
            if p.name == 'rot_center.x':
                cx = p.value
            if p.name == 'rot_center.y':
                cy = p.value
            if p.name == 'rot_center.z':
                cz = p.value
            if p.name == 'rot_radius':
                r = p.value
        kin_dict['typ'] = 'rotational'
        kin_dict['cx'] = cx
        kin_dict['cy'] = cy
        kin_dict['cz'] = cz
        kin_dict['r'] = r

    if data.name == 'prismatic':
        for p in data.params:
            if p.name == 'prismatic_dir.x':
                dx = p.value
            if p.name == 'prismatic_dir.y':
                dy = p.value
            if p.name == 'prismatic_dir.z':
                dz = p.value
        kin_dict['typ'] = 'prismatic'
        kin_dict['direc'] = [dx, dy, dz]

    kin_dict['got_model'] = True


##
# fit either a circle or a linear model.
# @param pts - 3xN np matrix of points.
# @return dictionary with kinematic params. (see model_cb)
def fit(pts, tup):
    kin_dict, track_pub = tup
    track_msg = TrackMsg()
    track_msg.header.stamp = rospy.get_rostime()
    track_msg.header.frame_id = '/'
    track_msg.track_type = TrackMsg.TRACK_POSITION_ONLY
    kin_dict['got_model'] = False
    for pt in pts.T:
        p = pt.A1.tolist()
        pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
        pose.position.x = p[0]
        pose.position.y = p[1]
        pose.position.z = p[2]
        track_msg.pose.append(pose)

    rospy.sleep(0.1)
    track_pub.publish(track_msg)
    while kin_dict['got_model'] == False:
        rospy.sleep(0.01)
    return kin_dict

def init_ros_node():
    rospy.init_node('kinematics_estimation_sturm')
    track_pub = rospy.Publisher('track', TrackMsg)
    kin_dict = {}
    track_msg = TrackMsg()
    rospy.Subscriber('model', ModelMsg, model_cb,
                     kin_dict)
    return kin_dict, track_pub


if __name__ == '__main__':
    init_ros_node()
    rospy.spin()

