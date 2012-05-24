import roslib; roslib.load_manifest('trf_learn')
import numpy as np
import pylab as pb
import hrl_lib.util as ut
import sys
import pdb
import geometry_msgs.msg as gm
import rospy
import tf.transformations as tr
import pylab as pb
import scipy.stats.morestats as mst
import itertools as it

def transforms_to_arrays(tlist):
    trans_list = []
    rotat_list = []

    for t in tlist:
        trans = t.transform.translation
        rotat = t.transform.rotation
        trans_arr = [trans.x, trans.y, trans.z]
        rotat_arr = [rotat.x, rotat.y, rotat.z, rotat.w]
        trans_list.append(trans_arr)
        rotat_list.append(rotat_arr)
    return np.matrix(trans_list).T, np.matrix(rotat_list).T

def transforms_to_pose_array(tlist):
    header = None
    poses = []
    for t in tlist:
        header = t.header
        pose = gm.Pose()
        pose.position.x = t.transform.translation.x
        pose.position.y = t.transform.translation.y
        pose.position.z = t.transform.translation.z
        pose.orientation.x = t.transform.rotation.x
        pose.orientation.y = t.transform.rotation.y
        pose.orientation.z = t.transform.rotation.z
        pose.orientation.w = t.transform.rotation.w
        poses.append(pose)
    return gm.PoseArray(header, poses)


def create_pose_arrows(points, angles, d=.02):
    adist = np.row_stack((np.cos(angles)*d, np.sin(angles)*d))
    a_at_distance = points + adist
    return it.chain.from_iterable(zip(np.row_stack((points[0,:], a_at_distance[0,:])).T.tolist(), 
                                      np.row_stack((points[1,:], a_at_distance[1,:])).T.tolist()))

def transforms_to_2d_pose_arrays(tlist):
    p2d = []
    angles = []
    for t in tlist:
        p2d.append([t.transform.translation.x, t.transform.translation.y])
        angles.append(tr.euler_from_quaternion([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]))
    return np.matrix(p2d).T, np.matrix(angles).T

if __name__ == '__main__':
    rospy.init_node('stats_05')
    apub = rospy.Publisher('aloc', gm.PoseArray)
    bpub = rospy.Publisher('bloc', gm.PoseArray)
    pathpub = rospy.Publisher('path', gm.PoseArray)

    fname = sys.argv[1]
    data = ut.load_pickle(fname)
    print 'locations', data['locations'], data['locations'][0].__class__
    print 'base poses', data['base_poses'][0][0], data['base_poses'][0][0].__class__
    print 'messages', len(data['messages']), data['messages'][0].__class__
    mats = [transforms_to_arrays(l) for l in data['base_poses']]
    covs = [np.cov(mat[0]) for mat in mats]
    for j, cov in enumerate(covs):
        print 'For location', j
        for i in range(3):
            print '  variance in dim', i, 'is', np.sqrt(cov[i,i])
    
    aposes = transforms_to_pose_array(data['base_poses'][0])
    bposes = transforms_to_pose_array(data['base_poses'][1])
    pathposes = transforms_to_pose_array(data['messages'])


    #r = rospy.Rate(5)
    #print 'publishing...'
    #while not rospy.is_shutdown():
    #    apub.publish(aposes)
    #    bpub.publish(bposes)
    #    pathpub.publish(pathposes)
    #    r.sleep()

    a2d_poses = transforms_to_2d_pose_arrays(data['base_poses'][0])
    b2d_poses = transforms_to_2d_pose_arrays(data['base_poses'][1])
    pat_poses = transforms_to_2d_pose_arrays(data['messages'])
    aangs = a2d_poses[1][2,:]
    bangs = b2d_poses[1][2,:]
    pat_angs = pat_poses[1][2,:]

    for label, ang_mat in zip(['a','b','path'], [aangs, bangs, pat_angs]):
        mean = mst.circmean(ang_mat[0,:].A1, high=np.pi, low=-np.pi)
        std = mst.circstd(ang_mat[0,:].A1, high=np.pi, low=-np.pi)
        print label, 'mean', np.degrees(mean), 'std', np.degrees(std)

    #At a distance of .5 meters what would be the variation.
    distance = 0.5
    adist = np.row_stack((np.cos(aangs)*distance, np.sin(aangs)*distance))
    bdist = np.row_stack((np.cos(bangs)*distance, np.sin(bangs)*distance))
    a_at_distance = a2d_poses[0] + adist
    b_at_distance = b2d_poses[0] + bdist
    a_cov_dist = np.cov(a_at_distance)
    b_cov_dist = np.cov(b_at_distance)

    dist_covs  = [np.cov(adist), np.cov(bdist)]
    comb_covs = [a_cov_dist, b_cov_dist]

    for i in range(2):
        orig_cov = covs[i]
        dist_cov = dist_covs[i]
        comb_cov = comb_covs[i]

        print '============='
        for i in range(2):
            print 'var %.6f+%.6f=%.6f (%.6f)' % (orig_cov[i,i], dist_cov[i,i], orig_cov[i,i]+dist_cov[i,i], comb_cov[i,i])
            print 'std %.6f %.6f %.6f' % (np.sqrt(orig_cov[i,i]), np.sqrt(dist_cov[i,i]), np.sqrt(comb_cov[i,i]))
        print ' '

    #pdb.set_trace()
    #deliver = it.chain.from_iterable(zip(np.row_stack((a2d_poses[0][0,:], a_at_distance[0,:])).T.tolist(), 
    #                                     np.row_stack((a2d_poses[0][1,:], a_at_distance[1,:])).T.tolist()))
    #pb.plot(*deliver, linestyle='-', marker='o')
    #pb.plot(*create_pose_arrows(a2d_poses[0], aangs, .02), linestyle='-', marker='o')
    #pb.plot(*create_pose_arrows(b2d_poses[0], bangs, .02), linestyle='-', marker='o')
    #pb.plot(*zip(a2d_poses[0].T.tolist(), a_at_distance.T.tolist()), linestyle='b-', marker='o')
    #pdb.set_trace()
    #select_idx = range(0,pat_poses[0].shape[1],15)
    #reduced_poses = pat_poses[0][:, select_idx]
    #pb.plot(*create_pose_arrows(reduced_poses, pat_angs, .02), linestyle='-')
    pb.plot(pat_poses[0][0,:].A1, pat_poses[0][1,:].A1, 'g-')
    pb.plot(a2d_poses[0][0,:].A1, a2d_poses[0][1,:].A1, 'bo')
    pb.plot(b2d_poses[0][0,:].A1, b2d_poses[0][1,:].A1, 'bo')
    
    pb.plot(a_at_distance[0,:].A1, a_at_distance[1,:].A1, 'ro')
    pb.plot(b_at_distance[0,:].A1, b_at_distance[1,:].A1, 'ro')

    #print a2d_poses[0][0,:]
    #print a2d_poses[0][1,:]
    pb.show()

    print 'done!'



