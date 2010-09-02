import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import tf.transformations as tr
import hrl_lib.tf_utils as tfu
import hrl_lib.util as ut
import tf

import geometry_msgs.msg as gm
import trigger_msgs.msg as trm
import sys
import math
import numpy as np

class MoveBase:
    def __init__(self):
        rospy.init_node('drive')
        self.tw_pub = rospy.Publisher('base_controller/command', gm.Twist)
        self.tl = tf.TransformListener()

    def go_ang(self, ang, speed):
        dt = math.radians(ang)

        if dt > 0:
            sign = -1
        elif dt < 0:
            sign = 1
        else:
            sign = 0

        self.tl.waitForTransform('base_footprint', 'odom_combined', rospy.Time(), rospy.Duration(10))
        p0_base = tfu.transform('base_footprint', 'odom_combined', self.tl)# \
        start_ang = tr.euler_from_matrix(p0_base[0:3, 0:3], 'sxyz')[2]
        r = rospy.Rate(100)
        dist_so_far = 0.

        last_ang = start_ang
        while not rospy.is_shutdown():
            pcurrent_base = tfu.transform('base_footprint', 'odom_combined', self.tl) #\
            current_ang = tr.euler_from_matrix(pcurrent_base[0:3, 0:3], 'sxyz')[2]
            dist_so_far = dist_so_far + (ut.standard_rad(current_ang - last_ang))
            if dt > 0 and dist_so_far > dt:
                rospy.loginfo('stopped! %f %f' % (dist_so_far, dt))
                break
            elif dt < 0 and dist_so_far < dt:
                rospy.loginfo('stopped! %f %f' % (dist_so_far, dt))
                break  
            elif dt == 0:  
                rospy.loginfo('stopped! %f %f' % (dist_so_far, dt))
                break

            tw = gm.Twist()
            tw.angular.z = math.radians(speed * sign)

            self.tw_pub.publish(tw)
            r.sleep()
            last_ang = current_ang

    def go_x(self, x, speed):
        print 'go x called!'
        vel = speed * np.sign(x)
        self.tl.waitForTransform('base_footprint', 'odom_combined', rospy.Time(), rospy.Duration(10))
        p0_base = tfu.transform('base_footprint', 'odom_combined', self.tl)
        r = rospy.Rate(100)

        while not rospy.is_shutdown():
            pcurrent_base = tfu.transform('base_footprint', 'odom_combined', self.tl)
            relative_trans = np.linalg.inv(p0_base) * pcurrent_base
            dist_moved = np.linalg.norm(relative_trans[0:3,3])
            print "%s" % str(dist_moved)
            
            if dist_moved > np.abs(x):
                rospy.loginfo('stopped! error %f' % (np.abs(dist_moved-np.abs(x))))
                break

            tw = gm.Twist()
            tw.linear.x = vel
            tw.linear.y = 0
            tw.linear.z = 0
            tw.angular.x = 0
            tw.angular.y = 0
            tw.angular.z = 0

            self.tw_pub.publish(tw)
            r.sleep()

if __name__ == '__main__':
    m = MoveBase()
    #m.go_ang(-390, 100)
    m.go_x(.2, .05)
