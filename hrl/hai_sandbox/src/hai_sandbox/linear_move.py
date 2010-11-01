import roslib; roslib.load_manifest('hai_sandbox')
import hrl_pr2_lib.pr2 as pr2
import hrl_pr2_lib.pr2_kinematics as pk
import pdb
import hrl_lib.util as ut
import pr2_gripper_reactive_approach.reactive_grasp as rgr
import pr2_gripper_reactive_approach.controller_manager as con
import numpy as np
import rospy
import hrl_lib.tf_utils as tfu
import tf.transformations as tr
import object_manipulator.convert_functions as cf
import math
from actionlib_msgs.msg import GoalStatus
import copy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import tf

class ActionType:
    def __init__(self, inputs, outputs):
        self.inputs = inputs
        self.outputs = outputs

class ParamType:
    def __init__(self, name, ptype, options=None):
        self.name = name
        self.ptype = ptype
        self.options = None

class Action:

    def __init__(self, name, params):
        self.name = name
        self.params = params

class BehaviorDescriptor:

    def __init__(self):
        self.descriptors = {
                            'twist':       ActionType([ParamType('angle', 'radian')], [ParamType('success', 'bool')]),
                            'linear_move': ActionType([ParamType('start_loc', 'se3'), 
                                                       ParamType('movement', 'r3'), 
                                                       ParamType('stop', 'discrete', ['pressure', 'pressure_accel'])], 
                                                      [ParamType('success', 'bool')]),
                            #'close_gripper': ActionType([,
                            #'open_gripper': k,
                            }

        start_location = (np.matrix([0.3, 0.15, 0.9]).T, np.matrix([0., 0., 0., 0.1]))
        movement       = np.matrix([.4, 0, 0.]).T
        self.seed = [Action('linear_move', [start_location, movement, 'pressure']),
                     Action('linear_move', [Action('current_location', [])])]
        self.run(self.seed)

    #def execute_action_list(self):

    #def run(self, seed):
    #    # search for pairs of perception operators and manipulation operators that would work
    #    population = 10
    #    seeds = []
    #    for i in range(population):
    #        aseed = copy.deepcopy(seed)
    #        # 'bool', 'radian', 'se3', 'r3', 'discrete', 
    #        new_seed_actions = []
    #        for action in aseed:

    #            if replace_action:
    #                pass

    #            if delete_action:
    #                pass
    #            
    #            if insert_action:
    #                #pick random action from descriptors list
    #                new_action = 
    #                new_seed_actions += new_action
    #                pass
    #            
    #            if perturb_parameter:
    #                num_params = len(action.params)
    #                rand_param_idx = ...
    #                self.descriptors[action.name].params[rand_param_idx]
    #                rand_param_types[rand_param_types]


    #            #can replace/delete/insert action
    #            #can pick a parameter and perturb it

    #    pdb.set_trace()
    #    print seed

class LaserPointerClient:
    def __init__(self, target_frame='/base_link', tf_listener=None):
        self.dclick_cbs = []
        self.point_cbs = []
        self.target_frame = target_frame
        self.laser_point_base = None
        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

        rospy.Subscriber('cursor3d', PointStamped, self.laser_point_handler)
        self.double_click = rospy.Subscriber('mouse_left_double_click', String, self.double_click_cb)

    def transform_point(self, point_stamped):
        point_head = point_stamped.point
        #Tranform into base link
        base_T_head = tfu.transform(self.target_frame, point_stamped.header.frame_id, self.tf_listener)
        point_mat_head = tfu.translation_matrix([point_head.x, point_head.y, point_head.z])
        point_mat_base = base_T_head * point_mat_head
        t_base, _ = tfu.matrix_as_tf(point_mat_base)
        return np.matrix(t_base).T
        
    def laser_point_handler(self, point_stamped):
        self.laser_point_base = self.transform_point(point_stamped)
        for f in self.point_cbs:
            f(self.laser_point_base)

    def double_click_cb(self, a_str):
        rospy.loginfo('Double CLICKED')
        if self.laser_point_base != None:
            for f in self.dclick_cbs:
                f(self.laser_point_base)
            self.laser_point_base = None

    def add_double_click_cb(self, func):
        self.dclick_cbs.append(func)

    def add_point_cb(self, func):
        self.point_cbs.append(func)


class Behaviors:
    def __init__(self, arm, tf_listener=None):
        rospy.init_node('linear_move', anonymous=True)
        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

        self.cman = con.ControllerManager(arm, self.tf_listener, using_slip_controller=1)
        self.reactive_gr = rgr.ReactiveGrasper(self.cman)

        self.cman.start_joint_controllers()
        self.cman.start_gripper_controller()
        self.timeout = 10.0

        #rospy.Subscriber('cursor3d', PointStamped, self.laser_point_handler)
        #self.double_click = rospy.Subscriber('mouse_left_double_click', String, self.double_click_cb)

    ##
    #  Move in a line from start_loc for distance & dir given by movement
    #
    #  @param start_loc
    #  @param movement
    #  @param stop either 'pressure' or 'pressure_accel'
    def linear_move(self, start_loc, movement, stop='pressure'):
        strans, srot = start_loc
        etrans = strans + movement
        #pdb.set_trace()

        stop_func = self._process_stop_option(stop)
        r1 = self._move_cartesian(strans, srot, stop_func, timeout=self.timeout, settling_time=5.0)
        if r1:
            return r1

        r2 = self._move_cartesian(etrans, srot, stop_func, timeout=self.timeout, settling_time=5.0)
        return r1 and r2

    def twist(self, angle):
        pos, rot = self.cman.return_cartesian_pose() # in base_link
        ax, ay, az = tr.euler_from_quaternion(rot) 
        nrot = tr.quaternion_from_euler(ax + angle, ay, az)
        #pdb.set_trace()
        stop_func = self._process_stop_option(stop)
        return self._move_cartesian(np.matrix(pos).T, np.matrix(nrot).T, stop_func, timeout=self.timeout, settling_time=5.0)

    def gripper_close(self):
        self.reactive_gr.compliant_close()

    def gripper_open(self):
        self.reactive_gr.cm.command_gripper(.1, -1, 1)

    def current_location(self):
        pos, rot = self.cman.return_cartesian_pose()
        return np.matrix(pos).T, np.matrix(rot).T

    def _process_stop_option(self, stop):
        if stop == 'none':
            stop_func = None

        elif stop == 'pressure':
            stop_func = self._tactile_stop_func

        elif stop == 'pressure_accel':
            print 'USING ACCELEROMETERS'
            self.cman.start_gripper_event_detector(timeout=self.timeout)
            stop_func = self._check_gripper_event

        return stop_func

    def _check_gripper_event(self):
        state = self.cman.get_gripper_event_detector_state()
        #action finished (trigger seen)
        if state not in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
            rospy.loginfo("place carefully saw the trigger, stopping the arm")
            return True
        else:
            return False

    def _tactile_stop_func(self):
        #stop if you hit a tip, side, back, or palm
        (left_touching, right_touching, palm_touching) = self.reactive_gr.check_guarded_move_contacts()
        #saw a contact, freeze the arm
        if left_touching or right_touching or palm_touching:
            rospy.loginfo("CONTACT made!")
            return True
        else:
            return False

    ##move the wrist to a desired Cartesian pose while watching the fingertip sensors
    #settling_time is how long to wait after the controllers think we're there
    def _move_cartesian(self, position, orienation, \
            stop_func, timeout = 3.0, settling_time = 0.5):
        pose_stamped = cf.create_pose_stamped(position.T.A1.tolist() + orienation.T.A1.tolist())
        rg = self.reactive_gr

        rg.check_preempt()

        #send the goal to the Cartesian controllers
        rospy.loginfo("sending goal to Cartesian controllers")
        (pos, rot) = cf.pose_stamped_to_lists(rg.cm.tf_listener, pose_stamped, 'base_link')
        rg.move_cartesian_step(pos+rot, timeout, settling_time)

        #watch the fingertip/palm sensors until the controllers are done and then some
        start_time = rospy.get_rostime()
        done_time = None
        stopped = False
        #print 'enterning loop'
        while(1):

            rg.check_preempt()
            if stop_func != None and stop_func():
                rg.cm.switch_to_joint_mode()
                rg.cm.freeze_arm()
                stopped = True
                break

            #check if we're actually there
            if rg.cm.check_cartesian_really_done(pose_stamped, .0025, .05):
                rospy.loginfo("actually got there")
                break

            #check if the controllers think we're done
            if not done_time and rg.cm.check_cartesian_done():
                rospy.loginfo("check_cartesian_done returned 1")
                done_time = rospy.get_rostime()

            #done settling
            if done_time and rospy.get_rostime() - done_time > rospy.Duration(settling_time):
                rospy.loginfo("done settling")
                break

            #timed out
            if timeout != 0. and rospy.get_rostime() - start_time > rospy.Duration(timeout):
                rospy.loginfo("timed out")
                break
        #print 'move returning'
        return stopped
        #return whether the left and right fingers were touching
        #return (left_touching, right_touching, palm_touching)


class LightSwitchBehaviorTest:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.behaviors = Behaviors('l', tf_listener=self.tf_listener)

        self.laser_listener = LaserPointerClient(tf_listener=self.tf_listener)
        self.laser_listener.add_double_click_cb(self.start)

        self.start_location = (np.matrix([0.3, 0.15, 0.9]).T, np.matrix([0., 0., 0., 0.1]))
        self.behaviors.linear_move(self.start_location, np.matrix([0,0,0.]).T)

    def start(self, point):
        start_location = (np.matrix([0.3, 0.15, 0.9]).T, np.matrix([0., 0., 0., 0.1]))

        #movement = np.matrix([.4, 0., 0.]).T
        print '===================================================================='
        print '===================================================================='
        print '                           Got point!'
        print 'CORRECTING', point.T
        point[0,0] = point[0,0] - .15
        print 'NEW', point.T
        movement = point - self.behaviors.current_location()[0]
        #pdb.set_trace()
        self.behaviors.linear_move(self.behaviors.current_location(), movement, stop='pressure_accel')
        #Adjust for fingertip
        print 'move direction', movement.T
        print 'point', point.T
        print 'ending location', self.behaviors.current_location()[0].T

        back_alittle = np.matrix([-.04, 0., 0.]).T
        self.behaviors.linear_move(self.behaviors.current_location(), back_alittle, stop='none')

        down = np.matrix([0., 0., -.2]).T
        self.behaviors.linear_move(self.behaviors.current_location(), down, stop='pressure_accel')

        back = np.matrix([-.02, 0., 0.]).T
        self.behaviors.linear_move(self.behaviors.current_location(), back, stop='none')
        #pdb.set_trace()
        #b.twist(math.radians(30.))

        #bd = BehaviorDescriptor()
        print 'DONE.'


    def run(self):
        print 'running'
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

    #def detect_event(self):
    #    self.behaviors.cman.start_gripper_event_detector(timeout=40.)
    #    stop_func = self.behaviors._tactile_stop_func
    #    while stop_func():

if __name__ == '__main__':
    l = LightSwitchBehaviorTest()
    l.run()





































        #pass
        #self.robot = pr2.PR2()
        #self.kin = pk.PR2Kinematics(self.robot.tf_listener)

    #def linear_move(self, start_location, direction, distance, arm):
    #    if arm == 'left':
    #        arm_kin = self.kin.left
    #    else:
    #        arm_kin = self.kin.right

    #    start_pose = arm_kin.ik(start_location)
    #    loc = start_location[0:3, 4]
    #    end_location = loc + distance*direction
    #    end_pose = arm_kin.ik(end_location)

    #    self.robot.left_arm.set_pose(start_pose, 5.)             #!!!
    #    self.robot.left_arm.set_pose(end_pose, 5.)               #!!!







            ##stop if you hit a tip, side, back, or palm
            #(left_touching, right_touching, palm_touching) = rg.check_guarded_move_contacts()
            ##saw a contact, freeze the arm
            #if left_touching or right_touching or palm_touching:
            #    rospy.loginfo("saw contact")
            #    rg.cm.switch_to_joint_mode()
            #    rg.cm.freeze_arm()
            #    break

    #import pdb
    #start_location = [0.34, 0.054, 0.87] + [0.015454981255042808, -0.02674860197736427, -0.012255429236635201, 0.999447577565171]
    #direction = np.matrix([1., 0., 0.]).T

    #self.reactive_l.move_cartesian_step(start_location, blocking = 1)
    #(left_touching, right_touching, palm_touching) = self.reactive_l.guarded_move_cartesian(grasp_pose, 10.0, 5.0)
        #self.cman_r     = con.ControllerManager('r')
        #self.reactive_r = rgr.ReactiveGrasper(self.cman_r)

        #self.cman_r.start_joint_controllers()
        #self.reactive_r.start_gripper_controller()
    
        #(pos, rot) = self.cman.return_cartesian_pose()
        #pdb.set_trace()
        #currentgoal = pos + rot
        #currentgoal[2] -= .05
        #self.reactive_l.move_cartesian_step(currentgoal, blocking = 1)
        #(left_touching, right_touching, palm_touching) = self.reactive_l.guarded_move_cartesian(grasp_pose, 10.0, 5.0)
        #exit()
        #end_loc = start_location + direction * distance
        #self.reactive_l.move_cartesian_step(start_loc, blocking = 1)
        #self.reactive_l.move_cartesian_step(end_loc, blocking = 1)
    #left_pose = b.robot.left.pose()
    #left_cart = ut.load_pickle('start_pose.pkl')
    #pdb.set_trace()
    #kin_sol = b.kin.left.ik(left_cart)
    #b.robot.left.set_pose(kin_sol, 5.)
    ##b.linear_move(left_cart)
    ##left_cart = b.kin.left.fk(left_pose)
    ##pdb.set_trace()
    #print left_cart

    #(pos, rot) = cm.return_cartesian_pose()
    #currentgoal = pos+rot
    #currentgoal[2] -= .05
    #rg.move_cartesian_step(currentgoal, blocking = 1)
    #exit()


#b.linear_move()
#cart_pose = kin.left.fk('torso_lift_link', 'l_wrist_roll_link', joints)
#kin.left.ik(cart_pose, 'torso_lift_link')


