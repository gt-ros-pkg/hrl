import roslib
roslib.load_manifest('trigger_msgs')
import sys
import rospy
import dynamic_reconfigure.client
import time
import trigger_msgs.msg
import geometry_msgs.msg as gm
import time
import sys

dist = float(sys.argv[1])
print dist
rospy.init_node("move_back", anonymous=True)
move_base = rospy.Publisher('simple_move_base', gm.Pose2D)
r = rospy.Rate(10)
p2d = gm.Pose2D()
p2d.x = dist
p2d.y = .15
print 'move_back -.6'
r.sleep()
move_base.publish(p2d)
r.sleep()
r.sleep()
r.sleep()
r.sleep()
r.sleep()

#import threading as tr

#class ProjectorWakeUp(threading.Thread):
#
#    def __init__(self):
#        threading.Thread.__init__(self)
#        self.projector = dynamic_reconfigure.client.Client('camera_synchronizer_node')
#
#    def run(self):
#        self.projector.update_configuration(projector_on)
#        time.sleep(.2)
#        self.projector.update_configuration(projector_off)
#        time.sleep(.05)
#        self.projector.update_configuration(projector_on)
#        time.sleep(.05)
#        self.projector.update_configuration(projector_off)
#        time.sleep(2)
#        self.projector.update_configuration(projector_on)

#head_up = rospy.Publisher("head_up", trigger_msgs.msg.Trigger, latch = True)
#head_down = rospy.Publisher("head_down", trigger_msgs.msg.Trigger, latch = True)
#head_down_up = rospy.Publisher("head_down_up", trigger_msgs.msg.Trigger, latch = True)
#arm_on = rospy.Publisher("light_on", trigger_msgs.msg.Trigger, latch = True)
#arm_off = rospy.Publisher("light_off", trigger_msgs.msg.Trigger, latch = True)
#
#left_initial_pose = rospy.Publisher("left_start", trigger_msgs.msg.Trigger, latch = True)
#left_initial_pose0 = rospy.Publisher("left_start2", trigger_msgs.msg.Trigger, latch = True)
#right_initial_pose0 = rospy.Publisher("right_initial_pose0", trigger_msgs.msg.Trigger, latch = True)
##right_initial_pose00 = rospy.Publisher("right_initial_pose00", trigger_msgs.msg.Trigger, latch = True)
#froo_froo = rospy.Publisher("froo_froo", trigger_msgs.msg.Trigger, latch = True)
#head_look_around = rospy.Publisher("head_look_around2", trigger_msgs.msg.Trigger, latch = True)
#
#both_arms_forward2 = rospy.Publisher("both_arms_forward2", trigger_msgs.msg.Trigger, latch = True)
#both_arms_fold2 = rospy.Publisher("both_arms_fold2", trigger_msgs.msg.Trigger, latch = True)
#both_arms_fold_end_pose = rospy.Publisher("both_arms_fold_end_pose", trigger_msgs.msg.Trigger, latch = True)
#head_turn = rospy.Publisher("head_turn", trigger_msgs.msg.Trigger, latch = True)
#
#arm_spin = rospy.Publisher("arm_spin", trigger_msgs.msg.Trigger, latch = True)
#raise_the_roof = rospy.Publisher("raise_the_roof", trigger_msgs.msg.Trigger, latch = True)
#head_up_full = rospy.Publisher("head_up_full", trigger_msgs.msg.Trigger, latch = True)
#head_down_full = rospy.Publisher("head_down_full", trigger_msgs.msg.Trigger, latch = True)

#hand_up = rospy.Publisher("hand_up", trigger_msgs.msg.Trigger, latch = True)
#hand_down = rospy.Publisher("hand_down", trigger_msgs.msg.Trigger, latch = True)
#left_initial_pose00 = rospy.Publisher("left_initial_pose00", trigger_msgs.msg.Trigger, latch = True)
# right_initial_pose00 = rospy.Publisher("right_initial_pose00", trigger_msgs.msg.Trigger, latch = True)
#right_initial_pose = rospy.Publisher("right_initial_pose", trigger_msgs.msg.Trigger, latch = True)

#projector_on = {'camera_reset': False,
#                'forearm_l_rate': 30.0,
#                'forearm_l_trig_mode': 1,
#                'forearm_r_rate': 30.0,
#                'forearm_r_trig_mode': 1,
#                'narrow_stereo_trig_mode': 2,
#                'projector_mode': 3,
#                'projector_pulse_length': 0.002,
#                'projector_pulse_shift': 0.0,
#                'projector_rate': 58.823529411764703,
#                'projector_tweak': 0.0,
#                'prosilica_projector_inhibit': False,
#                'stereo_rate': 29.411764705882351,
#                'wide_stereo_trig_mode': 2}
#
#
#projector_off = {'camera_reset': False,
#                    'forearm_l_rate': 30.0,
#                    'forearm_l_trig_mode': 1,
#                    'forearm_r_rate': 30.0,
#                    'forearm_r_trig_mode': 1,
#                    'narrow_stereo_trig_mode': 2,
#                    'projector_mode': 1,
#                    'projector_pulse_length': 0.002,
#                    'projector_pulse_shift': 0.0,
#                    'projector_rate': 58.823529411764703,
#                    'projector_tweak': 0.0,
#                    'prosilica_projector_inhibit': False,
#                    'stereo_rate': 29.411764705882351,
#                    'wide_stereo_trig_mode': 2}
#projector = dynamic_reconfigure.client.Client('camera_synchronizer_node')


#r = rospy.Rate(120/60.)
#time.sleep(.2)
#r.sleep()
#projector_animation = ProjectorWakeUp()
#i = -23
#while not rospy.is_shutdown():
#
#    print '------------', i, '-------------'
#    if i == -23:
#        print 'left_initial_pose0'
#        left_initial_pose0.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 1.5)), rospy.get_param("~event", ""))
#
#    if i == -12:
#        print 'left_initial_pose'
#        left_initial_pose.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 2.0)), rospy.get_param("~event", ""))
#
#    if i == -6:
#        print 'arm_on'
#        arm_on.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.3)), rospy.get_param("~event", ""))
#
#    if i == -5:
#        print 'head_look_around'
#        head_look_around.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.2)), rospy.get_param("~event", ""))
#
#    if i >= 9 and i <= 49:
#        arm_i = (i - 9)
#        if arm_i % 8 == 0:
#            print 'lights off'
#            arm_off.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", .8)), rospy.get_param("~event", ""))
#
#        if arm_i % 8 == 4:
#            print 'lights on'
#            arm_on.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", .8)), rospy.get_param("~event", ""))
#
#    if i >= 15 and i <= 34:
#        head_i = i - 15
#        if head_i % 4 == 0:
#            #Down
#            print 'down'
#            head_down_up.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.1)), rospy.get_param("~event", ""))
#            #head_up.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.0)), rospy.get_param("~event", ""))
#
#        if head_i % 4 == 2:
#            #Up
#            print 'up'
#            #head_down.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.8)), rospy.get_param("~event", ""))
#            #head_up.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.0)), rospy.get_param("~event", ""))
#            #hand_up.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.0)), rospy.get_param("~event", ""))
#
#    if i >= 40 and i <= 43:
#        head_i = i - 41
#        if head_i % 4 == 0:
#            head_down_up.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.1)), rospy.get_param("~event", ""))
#            print 'down'
#
#        if head_i % 4 == 2:
#            print 'up'
#            #head_up.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.8)), rospy.get_param("~event", ""))
#
#    ################################################################################
#    ### FREESTYLE
#    ################################################################################
#    if i == 23:
#        print 'right_initial_pose0'
#        right_initial_pose0.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.3)), rospy.get_param("~event", ""))
#
#    #if i == 24:
#    #    print 'right_initial_pose0'
#    #    right_initial_pose00.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 3.5)), rospy.get_param("~event", ""))
#
#    if i == 26:
#    #if i == 29:
#        print 'arm_spin'
#        arm_spin.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 3.0)), rospy.get_param("~event", ""))
#
#    #if i >= 29 and i < 37:
#    #    if ((i-29) % 9) == 0:
#    #        print 'Free style!'
#    #        froo_froo.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 2.0)), rospy.get_param("~event", ""))
#
#    if i == 42:
#    #if i == 45:
#        #ANOTHER FREESTYLE
#        print 'raise_the_roof'
#        raise_the_roof.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 3.0)), rospy.get_param("~event", ""))
#
#    ###############################################################################
#    ## Dancy
#    ###############################################################################
#    if i == 53:
#        print 'head_down'
#        print 'both_arms_forward2'
#        both_arms_forward2.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 2)), rospy.get_param("~event", ""))
#
#    if i == 56:
#        p2d = gm.Pose2D()
#        p2d.x = -.4
#        p2d.y = .15
#        move_base.publish(p2d)
#        head_down_full.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.5)), rospy.get_param("~event", ""))
#
#    if i == 61:
#        print 'head_up'
#        print 'both_arms_fold2'
#        head_up_full.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.4)), rospy.get_param("~event", ""))
#        both_arms_fold2.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.8)), rospy.get_param("~event", ""))
#
#    if i == 65:
#        p2d = gm.Pose2D()
#        p2d.y = 100.
#        p2d.theta = -390
#        move_base.publish(p2d)
#
#    if i == 77:
#        print 'both_arms_fold_end_pose'
#        print 'head_turn'
#        head_turn.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 7.0)), rospy.get_param("~event", ""))
#        both_arms_fold_end_pose.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.8)), rospy.get_param("~event", ""))
#
#    i = i+1
#    r.sleep()
#    if i == 80:
#        break

#projector.update_configuration(projector_off)


















    #if i == -12:
    #    left_initial_pose00.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 5)), rospy.get_param("~event", ""))
    #    right_initial_pose00.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 5)), rospy.get_param("~event", ""))

    #if i == 43:
    #    right_initial_pose.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 3.)), rospy.get_param("~event", ""))
