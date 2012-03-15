#
# Subscribe to ObjectDetection message and convert a pickle.
#

#
#

import roslib; roslib.load_manifest('modeling_forces')
import rospy

from checkerboard_detector.msg import ObjectDetection
import hrl_lib.util as ut
import hrl_lib.transforms as tr
from std_msgs.msg import Empty

import numpy as np
import time

def pose_cb(data, d):
    global t_pose_cb
    t_pose_cb = time.time()

    for obj in data.objects:
        p =  obj.pose.position
        pos = np.matrix([p.x, p.y, p.z]).T

        q =  obj.pose.orientation
        rot = tr.quaternion_to_matrix([q.x, q.y, q.z, q.w])
        t = data.header.stamp.to_sec()

        d[obj.type]['pos_list'].append(pos)
        d[obj.type]['rot_list'].append(rot)
        d[obj.type]['time_list'].append(t)

        print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
        print 'pos:', pos.T
        print 'rot:', rot
    
def got_trigger_cb(data, d):
    d['flag'] = True


if __name__ == '__main__':
    
    print 'Hello World.'
    rospy.init_node('checkerboard_poses_to_pickle', anonymous=False)

    topic_name = '/checkerdetector/ObjectDetection'
    d = {'mechanism': {}, 'hand': {}}
    d['mechanism'] = {'pos_list': [], 'rot_list': [], 'time_list': []}
    d['hand'] = {'pos_list': [], 'rot_list': [], 'time_list': []}

    rospy.Subscriber(topic_name, ObjectDetection, pose_cb, d)

    got_trigger_dict = {'flag': False}
    rospy.Subscriber('/checker_to_poses/trigger', Empty, got_trigger_cb,
                     got_trigger_dict)


    global t_pose_cb
    t_pose_cb = time.time()
    while got_trigger_dict['flag'] == False:
        time.sleep(0.05)
        if (time.time() - t_pose_cb) > 60.:
            raise RuntimeError('haven\'t received a pose_cb in 60 secs')

#    rospy.spin()
    print 'Number of poses:', len(d['mechanism']['time_list'])
    #ut.save_pickle(d, 'poses_dict_'+ut.formatted_time()+'.pkl')
    ut.save_pickle(d, 'poses_dict.pkl')



