#from sound_play.libsoundplay import SoundClient
import hrl_lib.tf_utils as tfu
from std_msgs.msg import String
import rospy
import tf

import subprocess as sb
import numpy as np
import geometry_msgs.msg as gms
import os

class LaserPointerClient:

    def __init__(self, target_frame='/base_link', tf_listener=None):
        self.dclick_cbs = []
        self.point_cbs = []
        self.target_frame = target_frame
        self.laser_point_base = None
        self.base_sound_path = (sb.Popen(["rospack", "find", "laser_interface"], stdout=sb.PIPE).communicate()[0]).strip()
        #self.sound = SoundClient()

        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

        rospy.Subscriber('cursor3d', gms.PointStamped, self.laser_point_handler)
        self.double_click = rospy.Subscriber('mouse_left_double_click', String, self.double_click_cb)
        os.system("aplay %s" % (self.base_sound_path + '/sounds/beep_beep.wav'))
        #self.sound.waveSound().play()


    def transform_point(self, point_stamped):
        point_head = point_stamped.point
        #Tranform into base link
        base_T_head = tfu.transform(self.target_frame, point_stamped.header.frame_id, self.tf_listener)
        point_mat_head = tfu.translation_matrix([point_head.x, point_head.y, point_head.z])
        point_mat_base = base_T_head * point_mat_head
        t_base, _ = tfu.matrix_as_tf(point_mat_base)
        return np.matrix(t_base).T
        
    def laser_point_handler(self, point_stamped):
        #self.sound.waveSound(self.base_sound_path + '/sounds/blow.wav').play()
        #os.system("aplay %s" % (self.base_sound_path + '/sounds/beeeeeep.wav'))
        self.laser_point_base = self.transform_point(point_stamped)
        for f in self.point_cbs:
            f(self.laser_point_base)

    def double_click_cb(self, a_str):
        rospy.loginfo('Double CLICKED')
        #self.sound.waveSound(self.base_sound_path + '/sounds/beep.wav').play()
        os.system("aplay %s" % (self.base_sound_path + '/sounds/beep_beep.wav'))
        #if self.laser_point_base != None:
        for f in self.dclick_cbs:
            f(self.laser_point_base)
        self.laser_point_base = None

    def add_double_click_cb(self, func):
        self.dclick_cbs.append(func)

    def add_point_cb(self, func):
        self.point_cbs.append(func)
