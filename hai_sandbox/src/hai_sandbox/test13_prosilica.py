import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import cv
import sys
import hrl_lib.util as ut
import hrl_lib.rutils as ru
import sensor_msgs.msg as sm
import pr2_msgs.msg as pm
import hai_sandbox.features as fea
import tf
import tf.transformations as tr
import hrl_lib.tf_utils as tfu
import numpy as np 
import features as fea
from cv_bridge import CvBridge, CvBridgeError
import scipy.spatial as sp


#Open bag file
#Register...
#need to transform pointcloud message to baseframe too


#Find surf features closest to contact point

#find contact points & transform gripper tip to base_frame
class ListenAndFindContactLocs:
    def __init__(self):#, pointcloud_msg):
        #rospy.init_node('contact3d')
        rospy.Subscriber('/pressure/l_gripper_motor', pm.PressureState, self.lpress_cb)
        self.ftip_frames = ['r_gripper_l_finger_tip_link',
                            'r_gripper_r_finger_tip_link',
                            'l_gripper_l_finger_tip_link',
                            'l_gripper_r_finger_tip_link']

        self.tflistener = tf.TransformListener()

        self.lmat0 = None
        self.rmat0 = None
        self.contact_locs = []
        self.base_frame = '/base_footprint'

        self.contact = False
        self.contact_stopped = False
        self.pointcloud_transform = None
        #self.pointcloud_msg = pointcloud_msg


    def lpress_cb(self, pmsg):
        #print 'called'
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
                print 'contact stopped'
                self.contact_stopped = True
            self.contact = False

            #Only get this transform after we've stopped making contact.
            #self.pointcloud_transform = tfu.transform(self.base_frame, self.pointcloud_msg.header.frame_id, self.tflistener)


if __name__ == '__main__':
    import sys

    proc_img_name = sys.argv[1]
    pickle_name = sys.argv[2]
    
    data_dict = ut.load_pickle(pickle_name) # ['camera_info', 'map_T_bf', 'pro_T_bf', 'points']
    proc_cam_info = ut.load_pickle('prosilica_caminfo.pkl')

    rospy.init_node('prosilica_set_view')
    points_pub = rospy.Publisher('/pc_snap_shot', sm.PointCloud)
    touchll_pub = rospy.Publisher('touch_ll', sm.PointCloud)
    touchlr_pub = rospy.Publisher('touch_lr', sm.PointCloud)
    proc_pub = rospy.Publisher('/prosilica/image_rect_color', sm.Image)
    cam_info = rospy.Publisher('/prosilica/camera_info', sm.CameraInfo)

    print 'pointcloud frame', data_dict['points'].header.frame_id
    point_cloud = ru.pointcloud_to_np(data_dict['points'])

    #skip this step if we have prerecorded pickle
    try:
        print 'loading contact_locs_proc.pkl'
        contact_locs = ut.load_pickle('contact_locs_proc.pkl')
    except Exception, e:
        print e
        print 'listening'         
        et = ListenAndFindContactLocs()
        r = rospy.Rate(10)
        while not rospy.is_shutdown() and not et.contact_stopped:
            r.sleep()
        contact_locs = et.contact_locs
        ut.save_pickle(et.contact_locs, 'contact_locs_proc.pkl')

    #Detect features, get 3d location for each feature
    print 'detecting features'
    proc_img = cv.LoadImage(proc_img_name)
    proc_gray = fea.grayscale(proc_img)
    sloc, sdesc = fea.surf(proc_gray)
    proc_surfed = fea.draw_surf(proc_img, sloc, (200, 0, 0))

    ######################################################################################
    # get 3d locations of surf features, get closest surf feature to gripper tips
    ######################################################################################
    #import pdb
    #pdb.set_trace()
    point_cloud_pro = data_dict['pro_T_bf'] * np.row_stack((point_cloud, 1+np.zeros((1, point_cloud.shape[1]))))
    #project 3d points to 2d
    point_cloud_2d = data_dict['camera_info'].project(point_cloud_pro[0:3,:])
    point_cloud_2d_tree = sp.KDTree(np.array(point_cloud_2d.T))

    #2d surf => 3d loc
    surf_loc3d = []
    for loc, lap, size, d, hess in sloc:
        idx = point_cloud_2d_tree.query(np.array(loc))[1]
        surf_loc3d.append(point_cloud[:, idx])
    surf_loc3d = np.column_stack(surf_loc3d)
    surf_loc_tree_bf = sp.KDTree(np.array(surf_loc3d.T))

    #get surf features closest to contact locs
    left_contact, right_contact = zip(*[(np.matrix(r[1][2]).T, np.matrix(r[1][3]).T) for r in contact_locs])
    left_contact = np.column_stack(left_contact)
    right_contact = np.column_stack(right_contact)
    mid_contact_bf = (left_contact[:,0] + right_contact[:,0]) / 2.
    #data_dict['pro_T_bf']  * np.row_stack((mid_contact_bf, np

    surf_closest_idx = surf_loc_tree_bf.query(np.array(mid_contact_bf.T))[1]
    surf_closest3d = surf_loc3d[:, surf_closest_idx]
    surf_closest_fea = sloc[surf_closest_idx]

    #draw this surf feature in image
    proc_surfed = fea.draw_surf(proc_surfed, [surf_closest_fea], (0,0,255))
    import pdb
    pdb.set_trace()
    cv.SaveImage('proc_surfed.png', proc_surfed )

    bridge = CvBridge()
    image_message = bridge.cv_to_imgmsg(proc_surfed, "bgr8")

    print 'init for viz'
    #left_contact = np.column_stack(left_contact)
    #right_contact = np.column_stack(right_contact)
    left_con_pc = ru.np_to_pointcloud(left_contact, '/base_footprint')
    right_con_pc = ru.np_to_pointcloud(right_contact, '/base_footprint')

    print 'publishing to rviz'
    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        points_pub.publish(data_dict['points'])
        touchll_pub.publish(left_con_pc)
        touchlr_pub.publish(right_con_pc)
        proc_pub.publish(image_message)
        cam_info.publish(proc_cam_info)
        r.sleep()

    #image_gray = fea.grayscale(image)
    #surf_keypoints, surf_descriptors = fea.surf(image_gray)
    #pointcloud_base = et.pointcloud_transform * np.row_stack((point_cloud, np.matrix(np.zeros(1, point_cloud.shape[1]))))
    ##TRANSFORM FROM POINTCLOUD TO CAMERA MISSING!!!!!!!!!!!!!!!!!!!!!!!!
    ##TRANSFORM FROM POINTCLOUD TO CAMERA MISSING!!!!!!!!!!!!!!!!!!!!!!!!
    ##TRANSFORM FROM POINTCLOUD TO CAMERA MISSING!!!!!!!!!!!!!!!!!!!!!!!!
    ##TRANSFORM FROM POINTCLOUD TO CAMERA MISSING!!!!!!!!!!!!!!!!!!!!!!!!
    #et.contact_locs













