import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import sensor_msgs.msg as sm
import hrl_lib.rutils as ru
import numpy as np
import pr2_msgs.msg as pm 
import tf
import hrl_lib.tf_utils as tfu
import tf.transformations as tr
import time
import hrl_lib.util as ut
              
def pointcloud_to_np(pc):
    plist = []
    for p in pc.points:
        plist.append([p.x, p.y, p.z])
    return np.matrix(plist).T


class ContactTipLocation:
    def __init__(self):
        rospy.init_node('contact3d')
        rospy.Subscriber('/pressure/l_gripper_motor', pm.PressureState, self.lpress_cb)
        rospy.Subscriber('/pressure/l_gripper_motor', pm.PressureState, self.rpress_cb)
        self.ftip_frames = ['r_gripper_l_finger_tip_link',
                            'r_gripper_r_finger_tip_link',
                            'l_gripper_l_finger_tip_link',
                            'l_gripper_r_finger_tip_link']

        self.tflistener = tf.TransformListener()
        self.lmat0 = None
        self.rmat0 = None
        self.contact_locs = []
        self.last_msg = None

    def lpress_cb(self, pmsg):
        #conv to mat
        lmat = np.matrix((pmsg.l_finger_tip)).T
        rmat = np.matrix((pmsg.r_finger_tip)).T
        if self.lmat0 == None:
            self.lmat0 = lmat
            self.rmat0 = rmat
            return

        #zero
        lmat = lmat - self.lmat0
        rmat = rmat - self.rmat0
   
        #touch detected
        if np.any(np.abs(lmat) > 250) or np.any(np.abs(rmat) > 250): #TODO: replace this with something more sound
            #Contact has been made!! look up gripper tip location
            to_frame = 'base_link'
            def frame_loc(from_frame):
                p_base = tfu.transform('base_footprint', from_frame, self.tflistener) \
                               * tfu.tf_as_matrix(([0., 0., 0., 1.], tr.quaternion_from_euler(0,0,0)))
                               #* tfu.tf_as_matrix((p.A1.tolist(), tr.quaternion_from_euler(0,0,0)))
                return tfu.matrix_as_tf(p_base)

            tip_locs = [frame_loc(n)[0] for n in self.ftip_frames]
            t = pmsg.header.stamp.to_time() 
            rospy.loginfo("contact detected at %.3f" % t)
            self.contact_locs.append([t, tip_locs])

        self.last_msg = time.time()
        rospy.loginfo('lpress_cb ' + str(np.max(rmat)) + ' ' + str(np.max(lmat)))

    def rpress_cb(self, pmsesg):
        pass
        #contact_mat(pmesg)


if __name__ == '__main__':
    import sys
    import pdb

    fname = sys.argv[1]
    scene = None
    # Use offline bag files
    # Load the pointcloud messages
    # Find out the variance in # of points
    # Select one as canonical
    i = 0
    for top, pc, t in ru.bag_iter(fname, ['/full_cloud']):
        # Want first message of at least this size
        if len(pc.points) > 20000:
            if i > 0:
                pdb.set_trace()
                scene = pointcloud_to_np(pc)
                break
            i = i + 1

    # Run online to get gripper tip locations from TF
    # Subscribe to pressure readings, find contact times & get gripper tip locations
    ctl = ContactTipLocation()
    r = rospy.Rate(10)
    print 'running contact tip recorder'
    while not rospy.is_shutdown():
        r.sleep()
        if ctl.last_msg != None and (time.time() - ctl.last_msg) > 60.0:
            break

    print 'saving pickle contact_locs.pkl'
    ut.save_pickle([scene, ctl.contact_locs], 'contact_locs.pkl')

    pdb.set_trace()
    print 'done.'

    # Use gripper tip locations to find out where in this pointcloud we are



















#class Contact3d:
#    def __init__(self):
#        rospy.init_node("contact_loc")
#        rospy.Subscriber('full_cloud', sm.PointCloud, self.point_cloud_cb)
#
#    def point_cloud_cb(self, pc_msg):
#        if len(pc_msg.points) < 1:
#            return
#        pointcloud_to_np(pc_msg.points)



    #c = Contact3d()
    #r = rospy.Rate(10)
    #print 'running'
    #while not rospy.is_shutdown():
    #    r.sleep()
