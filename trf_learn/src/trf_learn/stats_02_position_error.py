import roslib; roslib.load_manifest('trf_learn')
import rcommander_pr2_gui.pr2_utils as pru
import rospy
import tf 

import hrl_lib.tf_utils as tfu
import geometry_msgs as gm
import hrl_lib.util as ut

class PositionErrorExp:
    def __init__(self):
        ################################################################################
        #Which PR2 object to use??
        # HRL? RCommander? Perhaps the RCommander object needs to be released? We'll go
        # with the RCommander version and slowly refactor it out later... trf_learn
        # will need to have a dependency on RCommander.
        self.tf_listener = tf.TransformListener()
        self.pr2 = pru.PR2(tf_listener)
        self.path_to_action = 'raise_arm_pose_say_sleep_grip'
        self.graph_model = gm.GraphModel.load(self.path_to_action)
        self.pose_sub = rospy.Subscriber('/vrpn_tracked_object_1/pose', gm.TransformStamped, self.pose_cb)
        self.message = None

    def pose_cb(self, msg):
        self.message = msg

    #Copied over from application behaviors
    def move_base_planner(self, trans, rot):
        #pdb.set_trace()
        p_bl = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), np.matrix(trans).T)
        #self.turn_to_point(p_bl)
        rvalue = self.robot.base.set_pose(trans, rot, '/map', block=True)
        p_bl = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), np.matrix(trans).T)
        self.robot.base.move_to(p_bl[0:2,0], True)
        t_end, r_end = self.robot.base.get_pose()
        return rvalue==3, np.linalg.norm(t_end[0:2] - np.array(trans)[0:2])

    def run(self):
        #0. Load RCommander file, raise arm to pose, say, sleep, grip.
        state_machine = self.graph_model.create_state_machine(self.pr2)
        outcome = state_machine.execute()
        
        #1. Record two poses from map, a and b.
        raw_input("I'm going to record the current position. Press enter to continue.\n")
        a_loc = self.robot.base.get_pose()

        raw_input("Move the robot to the next position. Press enter to continue.\n")
        b_loc = self.robot.base.get_pose()

        #2. Move to a, record pose, move to b record current pose.
        tstring = time.strftime('%A_%m_%d_%Y_%I_%M%p')
        locations = [a_loc, b_loc]
        base_poses = [[], []]
        for i in range(10):
            for j in range(2):
                rvalue, dist = self.move_base_planner(*locations[j])
                ropsy.sleep(3)
                base_poses[j].append(self.message)
                ut.save_pickle({'locations':locations, 'base_poses': base_poses}, 
                                'stats_02_poses_%s.pkl' % tstring)
            print 'Finished iteration %d.' % i


