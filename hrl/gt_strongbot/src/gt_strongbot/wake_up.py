import roslib
roslib.load_manifest('trigger_msgs')
import sys
import rospy

import trigger_msgs.msg
import dynamic_reconfigure.client
import threading
import time
import geometry_msgs.msg as gm

rospy.init_node("wake_up", anonymous=True)

projector_on = {'camera_reset': False,
                'forearm_l_rate': 30.0,
                'forearm_l_trig_mode': 1,
                'forearm_r_rate': 30.0,
                'forearm_r_trig_mode': 1,
                'narrow_stereo_trig_mode': 2,
                'projector_mode': 3,
                'projector_pulse_length': 0.002,
                'projector_pulse_shift': 0.0,
                'projector_rate': 58.823529411764703,
                'projector_tweak': 0.0,
                'prosilica_projector_inhibit': False,
                'stereo_rate': 29.411764705882351,
                'wide_stereo_trig_mode': 2}

projector_off = {'camera_reset': False,
                    'forearm_l_rate': 30.0,
                    'forearm_l_trig_mode': 1,
                    'forearm_r_rate': 30.0,
                    'forearm_r_trig_mode': 1,
                    'narrow_stereo_trig_mode': 2,
                    'projector_mode': 1,
                    'projector_pulse_length': 0.002,
                    'projector_pulse_shift': 0.0,
                    'projector_rate': 58.823529411764703,
                    'projector_tweak': 0.0,
                    'prosilica_projector_inhibit': False,
                    'stereo_rate': 29.411764705882351,
                    'wide_stereo_trig_mode': 2}
projector = dynamic_reconfigure.client.Client('camera_synchronizer_node')
#move_base = rospy.Publisher('simple_move_base', gm.Pose2D)

blink_time = .2

print 'on'
projector.update_configuration(projector_on)
time.sleep(1)
print 'off'
projector.update_configuration(projector_off)
time.sleep(blink_time)

print 'on'
projector.update_configuration(projector_on)
time.sleep(blink_time)
print 'off'
projector.update_configuration(projector_off)
time.sleep(blink_time)

print 'on'
projector.update_configuration(projector_on)
time.sleep(blink_time)
print 'off'
projector.update_configuration(projector_off)
time.sleep(1)

print 'on'
projector.update_configuration(projector_on)
time.sleep(10)


#p2d = gm.Pose2D()
#p2d.x = .6
#p2d.y = .15
# move_base.publish(p2d)


#r = rospy.Rate(60/60.)
#projector_animation = ProjectorWakeUp()
#projector_animation.start()
#time.sleep(3)

































        #time.sleep(blink_time)
        #self.projector.update_configuration(self.projector_off)

        #self.projector.update_configuration(self.projector_on)
        #time.sleep(.2)

        #self.projector.update_configuration(self.projector_off)
        #time.sleep(.05)
        #self.projector.update_configuration(self.projector_on)
        #time.sleep(.05)

        #self.projector.update_configuration(self.projector_off)
        #time.sleep(.05)
        #self.projector.update_configuration(self.projector_on)
        #time.sleep(.05)

        #self.projector.update_configuration(self.projector_off)
        #time.sleep(2)
        #self.projector.update_configuration(self.projector_on)
        #time.sleep(3)
