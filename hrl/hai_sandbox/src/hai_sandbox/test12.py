import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import hrl_lib.rutils as ru
import sys
import sensor_msgs.msg as sm
import hrl_lib.util as ut
import hai_sandbox.features as fea
from cv_bridge.cv_bridge import CvBridge, CvBridgeError
import cv
import tf
import hrl_lib.tf_utils as tfu
from sensor_msgs.msg import CameraInfo
import numpy as np
import tf.transformations as tr
import scipy.spatial as sp

class ROSCameraCalibration:
    def __init__(self, channel):
        rospy.Subscriber(channel, CameraInfo, self.camera_info)
        self.has_msg = False

    def camera_info(self, msg):
        self.distortion = np.matrix(msg.D)
        self.K = np.reshape(np.matrix(msg.K), (3,3))
        self.R = np.reshape(np.matrix(msg.R), (3,3))
        self.P = np.reshape(np.matrix(msg.P), (3,4))
        self.w = msg.width
        self.h = msg.height
        self.frame = msg.header.frame_id
        self.has_msg = True

    ##
    # project 3D point into this camera 
    #   
    # @param p 3x1 matrix in given coord frame
    # @param tf_listener None if transformation not needed
    # @param from_frame None is default camera frame
    # @return 2x1 matrix
    def project(self, p, tf_listener=None, from_frame=None):
        if not(from_frame == None or from_frame == self.frame):
            p_cam = tfu.transform(self.frame, from_frame, tf_listener) \
                           * tfu.tf_as_matrix((p.A1.tolist(), tr.quaternion_from_euler(0,0,0)))
            trans, q = tfu.matrix_as_tf(p_cam)
            p = np.matrix(trans).T

        p = np.row_stack((p, np.matrix([1.])))
        pp = self.P * p
        pp = pp / pp[2,:]
        return pp[0:2,:]


class ImageProcess:

    def __init__(self, surf_name, contacts_name):
        forearm_cam_l = '/l_forearm_cam/image_rect_color'

        self.bridge = CvBridge()
        rospy.loginfo('loading %s' % surf_name)
        self.surf_data = ut.load_pickle(surf_name)
        rospy.loginfo('loading %s' % contacts_name)
        self.scene, self.contact_points = ut.load_pickle(contacts_name)
        self.surf_idx = None
        cv.NamedWindow('surf', 1)
        self.tflistener = tf.TransformListener()
        self.camera_geo = ROSCameraCalibration('/l_forearm_cam/camera_info')

        self.lmat0 = None
        self.rmat0 = None
        self.contact = False
        self.contact_stopped = False

        #rospy.Subscriber('/pressure/l_gripper_motor', pm.PressureState, self.lpress_cb)
        rospy.Subscriber(forearm_cam_l, sm.Image, self.image_cb)
        print 'ready'

    #def lpress_cb(self, msg):
    #def lpress_cb(self, pmsg):
    #    #conv to mat
    #    lmat = np.matrix((pmsg.l_finger_tip)).T
    #    rmat = np.matrix((pmsg.r_finger_tip)).T
    #    if self.lmat0 == None:
    #        self.lmat0 = lmat
    #        self.rmat0 = rmat
    #        return

    #    #zero
    #    lmat = lmat - self.lmat0
    #    rmat = rmat - self.rmat0
   
    #    #touch detected
    #    if np.any(np.abs(lmat) > 250) or np.any(np.abs(rmat) > 250): #TODO: replace this with something more sound
    #        self.contact = True
    #    else:
    #        if self.contact == True:
    #            self.contact_stopped = True
    #        self.contact = False
    #        #Contact has been made!! look up gripper tip location
    #        #to_frame = 'base_link'
    #        #def frame_loc(from_frame):
    #        #    p_base = tfu.transform('base_footprint', from_frame, self.tflistener) \
    #        #                   * tfu.tf_as_matrix(([0., 0., 0., 1.], tr.quaternion_from_euler(0,0,0)))
    #        #                   #* tfu.tf_as_matrix((p.A1.tolist(), tr.quaternion_from_euler(0,0,0)))
    #        #    return tfu.matrix_as_tf(p_base)

    #        #tip_locs = [frame_loc(n)[0] for n in self.ftip_frames]
    #        #t = pmsg.header.stamp.to_time() 
    #        #rospy.loginfo("contact detected at %.3f" % t)
    #        #self.contact_locs.append([t, tip_locs])

    #    #self.last_msg = time.time()
    #    rospy.loginfo('lpress_cb ' + str(np.max(rmat)) + ' ' + str(np.max(lmat)))

    def image_cb(self, msg):
        image_time = msg.header.stamp.to_time()
        image      = self.bridge.imgmsg_to_cv(msg, 'bgr8')
        if self.surf_idx == None:
            for i, d in enumerate(self.surf_data):
                t, sdata = d
                if image_time == t:
                    self.surf_idx = i
                    break
        else:
            self.surf_idx = self.surf_idx + 1

        stime, sdata = self.surf_data[self.surf_idx]
        if stime != image_time:
            print 'surf time != image_time'

        surf_keypoints, surf_descriptors = sdata 
        nimage = fea.draw_surf(image, surf_keypoints, (255,0,0))
        cv.ShowImage('surf', nimage)
        cv.WaitKey(10)

        # Track and give 3D location to features.
        ## Project 3D points into this frame (need access to tf => must do online or from cache)
        ##              camera_T_3dframe at ti
        scene2d = self.camera_geo.project(self.scene, self.tflistener, 'base_footprint')
        scene2d_tree = sp.KDTree(np.array(scene2d.T))

        ## Find features close to these 2d points
        for loc, lap, size, d, hess in surf_keypoints:
            idx = scene2d_tree.query(np.array(loc))[1]
            orig3d = self.scene[:, idx]

        ## Get features closest to the contact point
        ## stop running if contact has stopped
        if self.contact:
        #TODO: let's put this tracking on hold..


if __name__ == '__main__':
    surf_name = sys.argv[1]
    contacts_name = sys.argv[2]
    rospy.init_node('test12')
    ip = ImageProcess(surf_name, contacts_name)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()


#raw_video_bag = '2010-07-22-20-34-59/2010-07-22-20-34-59.bag'
#'2010-07-22-20-34-59.surf_pkl'

#Take all features in this video and map them to 3D surfaces.

    #Record features, pose of camera to do transformation...
    # From feature 2D point F2D
    # Project 3D points into this frame (need access to tf => must do online or from cache)
    #              camera_T_3dframe at ti
    #     2dpoints = camera_T_3dframe * 3dpoints
    # (Every frame has been tracked! just retrieve the appropriate feature from pickle.)
    
    # ("Offline phase")
    # grab nearest 3D point in image space
    #     2dpoints.query(f2d)
    # store this 3d location with this feature
    # 
    #Track features using rudimentary tracker (from laser interface)

#load pickle, this should match with video we're going to get.
#tracked_features_fname = sys.argv[1]
#tracked_features = ru.load_pickle(tracked_features_fname)
