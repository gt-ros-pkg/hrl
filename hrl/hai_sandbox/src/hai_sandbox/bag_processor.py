import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import sys
import hrl_lib.util as ut
import hrl_lib.rutils as ru

import tf
import hrl_lib.tf_utils as tfu
import tf.transformations as tr

from multiprocessing import Process
import time
import os
import numpy as np
import math
import pdb

def segment_msgs(time_segments, msgs):
    segs = []
    for segment in time_segments:
        start = segment[0]
        endt  = segment[1]
        sx = 0
        ex = len(msgs)

        if start != 'start':
            for i, m in enumerate(msgs):
                if m.header.stamp.to_time() < start:
                    sx = i
                else:
                    break

        if endt == 'end':
            ex = len(msgs)
            for i, m in enumerate(msgs[sx:]):
                if m.header.stamp.to_time() > endt:
                    ex = i + start
                    break

        #pdb.set_trace()
        seg = msgs[sx:ex]
        segs.append(msgs[sx: ex])
    return segs

##
# Find times where contact has been made
#
# @return array of locations where contact has been made, array of times for each location where contact has been made (can be duplicated)
def find_contact_times(left_mat, right_mat, times, thres):
    left_mat = left_mat - left_mat[:, 0] 
    right_mat = right_mat - right_mat[:,0]
    
    #When/where did contact happen? 
    #TODO: we are assuming just one finger of one arm here!
    #pdb.set_trace()
    loc_r, time_c = np.where(np.abs(left_mat) > thres)
    times_contact = times[time_c.A1]
    unique_times = np.array(np.sort(list(set(times_contact.tolist()))))
    #return (loc_r, times_contact)

    return unique_times


def playback_bag(bag_name):
    os.system('rosbag play %s' % bag_name)


class JointMsgConverter:
    def __init__(self):
        self.name_dict = None
        self.joint_groups = ut.load_pickle('link_names.pkl')
        #self.joint_groups['rarm']      = rospy.get_param('/r_arm_controller/joints')
        #self.joint_groups['larm']      = rospy.get_param('/l_arm_controller/joints')
        #self.joint_groups['head_traj'] = rospy.get_param('/head_traj_controller/joints')
        #self.joint_groups['torso']     = rospy.get_param('/torso_controller/joints')

    def msgs_to_dict(self, msgs):
        converted = []
        for i in range(len(msgs)):
            msg = msgs[i]
            if self.name_dict == None:
                self.name_dict = {}
                for i, n in enumerate(msg.name):
                    self.name_dict[n] = i 

                self.joint_idx = {}
                for group in self.joint_groups.keys():
                    self.joint_idx[group] = [self.name_dict[name] for name in self.joint_groups[group]]

            joint_poses = {}
            joint_vel = {}
            joint_eff = {}
            #extract data for pose, vel, eff
            for d, data in zip([joint_poses, joint_vel, joint_eff], [msg.position, msg.velocity, msg.effort]):
                #look up values for each joint group
                dmat = np.matrix(data).T
                for group in self.joint_groups.keys():
                    d[group] = dmat[self.joint_idx[group], 0]
            converted.append({'poses': joint_poses, 'vels': joint_vel, 'efforts': joint_eff, 'time': msg.header.stamp.to_time()})

        #Take out wrapping of forearm & wrist
        for group in ['rarm', 'larm']:
            for i in range(len(self.joint_groups[group])):
                joint = self.joint_groups[group][i]
                if 'forearm_roll' in joint or 'wrist_roll' in joint:
                    delta = msgs[1].position[i] - msgs[0].position[i]
                    realdelta = delta % (2 * math.pi)
                    if realdelta >= math.pi:
                        realdelta -= 2 * math.pi
                    correction = delta - realdelta
                    for j in range(1, len(msgs)):
                        converted[j]['poses'][group][i,0] -= correction
                        #msgs[j].positions[i] -= correction
        return converted





if __name__ == '__main__':
    import pylab as pb

    arm_used = 'left'
    full_bag_name = sys.argv[1]
    bag_path, bag_name_ext = os.path.split(full_bag_name)
    filename, ext = os.path.splitext(bag_name_ext)
    
    ###############################################################################
    # Playback the bag
    bag_playback = Process(target=playback_bag, args=(full_bag_name,))
    bag_playback.start()

    ###############################################################################
    ## Listen for transforms
    rospy.init_node('bag_proceessor')

    tl = tf.TransformListener()
    tl.waitForTransform('map', 'base_footprint', rospy.Time(), rospy.Duration(10))

    # Extract the starting location
    p_base = tfu.transform('map', 'base_footprint', tl) \
            * tfu.tf_as_matrix(([0., 0., 0., 1.], tr.quaternion_from_euler(0,0,0)))
    t, r = tfu.matrix_as_tf(p_base)
    pose_base = (t, r)
    if bag_playback.is_alive():
        rospy.loginfo('terminating playback process')
        bag_playback.terminate()
        time.sleep(1)
        rospy.loginfo('playback process terminated? %s' % str(not bag_playback.is_alive()))


    #print 'got loc %.2f %.2f %.2f'% (t[0], t[1], t[2])
    #loc_fname = '%s_loc.pkl' % os.path.join(bag_path, filename)
    #print 'saving to %s' % loc_fname
    #ut.save_pickle((t,r), loc_fname)

    ###############################################################################
    #Read bag
    converter = JointMsgConverter()
    rospy.loginfo('opening bag')
    topics_dict = ru.bag_sel(full_bag_name, ['/joint_states', '/l_cart/command_pose', 
                                             '/r_cart/command_pose', '/torso_controller/state',
                                             '/pressure/l_gripper_motor', '/pressure/r_gripper_motor'])

    ## Select the arm that has been moving, segment joint states based on contact states.
    if arm_used == 'left':
        pressures = topics_dict['/pressure/l_gripper_motor']
    elif arm_used == 'right':
        pressures = topics_dict['/pressure/r_gripper_motor']
    else:
        raise RuntimeError('arm_used invalid')

    rospy.loginfo('Finding contact times')
    left_f, right_f, ptimes = ru.pressure_state_to_mat(pressures['msg'])
    contact_times = find_contact_times(left_f, right_f, ptimes, 250)
    time_segments = [['start', contact_times[0]], [contact_times[0], 'end']]

    pressure_lseg = segment_msgs(time_segments, topics_dict['/pressure/l_gripper_motor']['msg'])
    pressure_rseg = segment_msgs(time_segments, topics_dict['/pressure/r_gripper_motor']['msg'])

    lcart_seg = segment_msgs(time_segments, topics_dict['/l_cart/command_pose']['msg'])
    rcart_seg = segment_msgs(time_segments, topics_dict['/r_cart/command_pose']['msg'])

    #print contact_times - contact_times[0]
    #print contact_times[1:] - contact_times[:-1]
    #pb.plot(contact_times-contact_times[0], 'g.')
    #pb.show()

    #Find the first robot pose
    ## Convert from joint state to dicts
    joint_states = topics_dict['/joint_states']['msg']
    j_segs     = segment_msgs(time_segments, topics_dict['/joint_states']['msg'])
    jseg_dicts = [converter.msgs_to_dict(seg) for seg in j_segs]
    j0_dict    = jseg_dicts[0][0]
    
    #converter.msg_to_dict(j_segs[0][0])
    #jseg_dicts = [[converter.msg_to_dict(j_msg) for j_msg in seg] for seg in j_segs]
    
    ###############################################################################
    movement_states = []
    for i, seg in enumerate(time_segments):
        name = "state_%d" % i
        start_times = [lcart_seg[i][0].header.stamp.to_time(), 
                       rcart_seg[i][0].header.stamp.to_time(), 
                       jseg_dicts[i][0]['time'],
                       pressure_lseg[i][0].header.stamp.to_time(), 
                       pressure_rseg[i][0].header.stamp.to_time()]

        sdict = {'name': name,
                 'start_time': np.min(start_times),
                 'cartesian': [lcart_seg[i], rcart_seg[i]],
                 'joint_states': jseg_dicts[i],
                 'pressure': [pressure_lseg[i], pressure_rseg[i]]
                 } 

        movement_states.append(sdict)


    ###store in a dict
    data = {'base_pose': pose_base, 
            'robot_pose': j0_dict,
            'arm': arm_used,
            'movement_states': None}

    processed_bag_name = '%s_processed.pkl' % os.path.join(bag_path, filename)
    rospy.loginfo( 'saving to %s' % processed_bag_name )
    ut.save_pickle(data, processed_bag_name)
    bag_playback.join()
    rospy.loginfo('finished!')

# pose_base = [t, r], t is len3, r is len4
# j0_dict {'poses': joint_poses, 'vels': joint_vel, 'efforts': joint_eff, 'time': msg.header.stamp.to_time()}
#         with each entry having keys: ['rarm'] ['larm'] ['head_traj'] ['torso']    
# arm is a string {'left', 'right'}
# movement_states
#       'name'
#       'start_time'
#       'cartesian'
#       'joint_states'
#       'pressure'


























            #'movement_states': [{'name': #,
            #                     'cartesian':#,
            #                     'joint_states': # j_dicts, j_times
            #                     'pressure': #,
            #                     }]
            #                      #}
    
    ##ut.save_pickle(data, extracted_name)


    #if len(joint_states) <= 1:
    #    raise RuntimeError('Not enough joint state messages.  Got %d messages.' % len(joint_states))
    #joint_states)
