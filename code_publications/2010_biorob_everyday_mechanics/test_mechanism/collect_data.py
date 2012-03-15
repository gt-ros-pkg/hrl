
import numpy as np
import time

import roslib; roslib.load_manifest('modeling_forces')
import rospy

from std_msgs.msg import Empty
from checkerboard_detector.msg import ObjectDetection

import hrl_lib.util as ut
import hrl_lib.transforms as tr
import force_torque.FTClient as ftc

from opencv.cv import *
from opencv.highgui import *
import cameras.dragonfly as dr

def pose_cb(data, d):
    if len(data.objects) != 2:
        return

    for obj in data.objects:
        p =  obj.pose.position
        pos = np.matrix([p.x, p.y, p.z]).T

        q =  obj.pose.orientation
        rot = tr.quaternion_to_matrix([q.x, q.y, q.z, q.w])
        t = data.header.stamp.to_sec()

        d[obj.type]['pos'] = pos
        d[obj.type]['rot'] = rot

def init_ft_client():
    client = ftc.FTClient('/force_torque_ft2')
    ft = client.read(fresh=True, with_time_stamp=False)
    return client

##
# get number from command line. Check if the number is of the
# appropriate dataype.
# @param dtype - int, float etc. 
def get_number(prompt, dtype):
    no_number = True
    number = -9999.0
    while no_number:
        print prompt
        str = raw_input()
        try:
            number = dtype(str)
            no_number = False
        except exceptions.ValueError, e:
            print 'That was not a valid %s! Try again.'%(dtype.__str__)

    return number


if __name__ == '__main__':

    rospy.init_node('mechanism_test_node', anonymous=False)
    ft_client = init_ft_client()

    current_pose_d = {'mechanism': {}, 'hand': {}}
    current_pose_d['mechanism'] = {'pos': None, 'rot': None}
    current_pose_d['hand'] = {'pos': None, 'rot': None}

    pose_d = {'mechanism': {}, 'hand': {}}
    pose_d['mechanism'] = {'pos_list': [], 'rot_list': [], 'time_list': []}
    pose_d['hand'] = {'pos_list': [], 'rot_list': [], 'time_list': []}

    topic_name = '/checkerdetector/ObjectDetection'
    #rospy.Subscriber(topic_name, ObjectDetection, pose_cb, pose_d)
    rospy.Subscriber(topic_name, ObjectDetection, pose_cb,
                     current_pose_d)

    camera_name = 'remote_head'
    cam = dr.dragonfly2(camera_name)
    cam.set_frame_rate(30)
    cam.set_auto()
    for i in range(10):
        im = cam.get_frame()

    measured_force_list = []
    spring_scale_list = []
    time_list = []

    i = 0
    while True:
        print 'hit "a" to take a reading, e to exit'
        str = raw_input()
        if str == 'a':
            ft = ft_client.read(fresh=True, with_time_stamp=False)
            measured_force_list.append(ft.A1.tolist())
            time_list.append(time.time())
            im = cam.get_frame()
            im = cam.get_frame()
            nm = '%05d.png'%i
            cvSaveImage(nm, im)
            i += 1

#            for obj in ['hand', 'mechanism']:
#                pose_d[obj]['pos_list'].append(current_pose_d[obj]['pos'])
#                pose_d[obj]['rot_list'].append(current_pose_d[obj]['rot'])

            f = get_number('Enter the spring scale reading', dtype = float)
            spring_scale_list.append(f)
        elif str == 'e':
            print 'Exiting ...'
            break

    ft_dict = {}
    ft_dict['ft_list'] = measured_force_list
    ft_dict['time_list'] = time_list
    fname = 'ft_log_'+ut.formatted_time()+'.pkl'
    ut.save_pickle(ft_dict, fname)

#    pose_d['hand']['time_list'] = time_list
#    pose_d['mechanism']['time_list'] = time_list
#    ut.save_pickle(pose_d, 'poses_dict.pkl')

    ut.save_pickle(spring_scale_list,
                   'spring_scale_'+ut.formatted_time()+'.pkl')



