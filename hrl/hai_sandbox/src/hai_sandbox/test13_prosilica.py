import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import cv
import sys
import hrl_lib.util as ut
import hrl_lib.rutils as ru
import hai_sandbox.features as fea


#Open bag file
#Register...
#need to transform pointcloud message to baseframe too


#Find surf features closest to contact point

#find contact points & transform gripper tip to base_frame
class ExtractTransforms:
    def __init__(self, pointcloud_msg):
        rospy.init_node('contact3d')
        rospy.Subscriber('/pressure/r_gripper_motor', pm.PressureState, self.rpress_cb)
        self.ftip_frames = ['r_gripper_l_finger_tip_link',
                            'r_gripper_r_finger_tip_link',
                            'l_gripper_l_finger_tip_link',
                            'l_gripper_r_finger_tip_link']

        self.tflistener = tf.TransformListener()

        self.lmat0 = None
        self.rmat0 = None
        self.contact_locs = []
        self.base_frame = 'base_footprint'

        self.contact = False
        self.contact_stopped = False
        self.pointcloud_transform = None
        self.pointcloud_msg = pointcloud_msg


    def rpress_cb(self, pmsg):

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
            def frame_loc(from_frame):
                p_base = tfu.transform(self.base_frame, from_frame, self.tflistener) \
                               * tfu.tf_as_matrix(([0., 0., 0., 1.], tr.quaternion_from_euler(0,0,0)))
                return tfu.matrix_as_tf(p_base)
            tip_locs = [frame_loc(n)[0] for n in self.ftip_frames]
            t = pmsg.header.stamp.to_time() 
            rospy.loginfo("contact detected at %.3f" % t)
            self.contact_locs.append([t, tip_locs])
            self.contact = True
        else:
            if self.contact == True:
                self.contact_stopped = True
            self.contact = False

            #Only get this transform after we've stopped making contact.
            self.pointcloud_transform = tfu.transform(self.base_frame, self.pointcloud_msg.header.frame_id, self.tflistener)


if __name__ == '__main__':
    import sys
    #import pdb

    #pdb.set_trace()
    image_name = sys.argv[1]
    pointcloud_name = sys.argv[2]
    
    pointcloud_msg = ut.load_pickle(pointcloud_name)
    image = cv.LoadImage(image_name)
    point_cloud = pointcloud_to_np(pointcloud_msg)

    r = rospy.Rate(10)
    et = ExtractTransforms()
    while not rospy.is_shutdown() and not et.contact_stopped:
        r.sleep()

    image_gray = fea.grayscale(image)
    surf_keypoints, surf_descriptors = fea.surf(image_gray)
    pointcloud_base = et.pointcloud_transform * np.row_stack((point_cloud, np.matrix(np.zeros(1, point_cloud.shape[1]))))
    #TRANSFORM FROM POINTCLOUD TO CAMERA MISSING!!!!!!!!!!!!!!!!!!!!!!!!
    #TRANSFORM FROM POINTCLOUD TO CAMERA MISSING!!!!!!!!!!!!!!!!!!!!!!!!
    #TRANSFORM FROM POINTCLOUD TO CAMERA MISSING!!!!!!!!!!!!!!!!!!!!!!!!
    #TRANSFORM FROM POINTCLOUD TO CAMERA MISSING!!!!!!!!!!!!!!!!!!!!!!!!
    et.contact_locs













