import roslib; roslib.load_manifest('hai_sandbox')
import hrl_lib.util as ut
import sys
import tf.transformations as tr
import opencv
import sensor_msgs.msg as sm


def points_in_view(points, cal_obj):
    valid_indices = np.where(np.multiply(np.multiply(points[0,:] > 0, points[0,:] < cal_obj.w-.6),
                                         np.multiply(points[1,:] > 0, points[1,:] < cal_obj.h-.6)))[1].A1
    return points[:, valid_indices]


##
#
# @return 2xn int matrix of 2d coordinates
# @return 3xn int matrix of rgb values
def laser_point_intensity(points_iframe, image, cal_obj):
    p2d = np.array(np.round(points_in_view(points_iframe, cal_ob)), dtype='int')
    imagea = np.asarray(image)
    intensity_channels = imagea[p2d[0,:].A1, p2d[1,:].A1, :]
    return np.matrix(p2d), np.matrix(intensity_channels).T


if __name__ == '__main__':
    rospy.init_node('better_recognize3d')
    highres_cloud_pub = rospy.Publisher('colorcloud_highres', sm.PointCloud)

    fname = sys.argv[1]
    data_pkl = ut.load_pickle(fname)

    points_pf = tr.inverse_matrix(data_pkl['laser_T_bf']) * data_pkl['points']
    points_cam = data_pkl['pro_T_bf'] * points_bf
    points_valid, colors = laser_point_intensity(points_cam, cv.LoadImageM(data_pkl['high_res']), data_pkl['prosilica_cal'])

    #calculate values of intensity

    #publish mapped cloud to verify
    pc_msg = ru.np_to_rgb_pointcloud(points_valid, colors, 'high_def_optical_frame')
    highres_cloud_pub.publish(pc_msg)

    r = rospy.rate(10)
    for i in range(10):
        if not rospy.is_shutdown():
            r.sleep()














