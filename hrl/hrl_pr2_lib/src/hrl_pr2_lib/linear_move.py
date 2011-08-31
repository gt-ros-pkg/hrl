import roslib; roslib.load_manifest('hrl_pr2_lib')
import rospy

import pr2_gripper_reactive_approach.reactive_grasp as rgr
import pr2_gripper_reactive_approach.controller_manager as con
import object_manipulator.convert_functions as cf

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from pr2_gripper_sensor_msgs.msg import PR2GripperEventDetectorGoal

#import collision_monitor as cmon
#import hrl_pr2_lib.devices as de
import hrl_lib.tf_utils as tfu
import hrl_pr2_lib.pressure_listener as pm
import numpy as np
import tf
import pdb
import time
import hrl_lib.util as ut
import math


class RobotSafetyError(Exception):
    def __init__(self, value):
        self.parameter = value

    def __str__(self):
        return repr(self.parameter)

## NOT THREAD SAFE
class ArmStoppedDetector:
    def __init__(self, pos_thres=0.0001, rot_thres=0.001, time_settle=1., hz=30., n_step=3000):
        self.pos_diff = []
        self.rot_diff = []
        self.times = []

        self.last_pos = None
        self.last_t = None
        self.n_step = n_step
        self.hz = hz
        self.time_settle = time_settle

        self.pos_thres = pos_thres
        self.rot_thres = rot_thres
        self.start_time = time.time()

    def record_diff(self, loc_mat):
        cur_t = time.time() - self.start_time
        if self.last_t == None or (cur_t - self.last_t) > (1./self.hz):
            pos, rot = tfu.matrix_as_tf(loc_mat)
            if self.last_pos != None:
                self.pos_diff.append(np.linalg.norm(np.matrix(pos) - np.matrix(self.last_pos[0])))

                lp = np.array(self.last_pos[1]) / np.linalg.norm(self.last_pos[1])
                r  = np.array(rot) / np.linalg.norm(rot)
                #pdb.set_trace()
                self.rot_diff.append(np.linalg.norm(lp - r))
                self.times.append(cur_t)

                sidx = len(self.pos_diff) - self.n_step
                self.pos_diff = self.pos_diff[sidx:]
                self.pose_diff = self.rot_diff[sidx:]

            self.last_pos = tfu.matrix_as_tf(loc_mat)
            self.last_t = cur_t

    def is_stopped(self):
        cur_t = time.time() - self.start_time
        pos_over_thres_idx = np.where(np.array(self.pos_diff) > self.pos_thres)[0]
        rot_over_thres_idx = np.where(np.array(self.rot_diff) > self.rot_thres)[0]
        if len(pos_over_thres_idx) > 0 or len(rot_over_thres_idx) > 0:
            max_times = []
            if len(pos_over_thres_idx) > 0:
                max_times.append(self.times[pos_over_thres_idx[-1]])
            if len(rot_over_thres_idx) > 0:
                max_times.append(self.times[rot_over_thres_idx[-1]])
            tmax = np.max(max_times)
            if (cur_t - tmax) > self.time_settle:
                print cur_t - tmax, tmax, self.time_settle, max_times
                return True
            else:
                return False
        elif len(self.pos_diff) > (self.time_settle * self.hz):
            return True
        else:
            return False


        #if (np.any(np.array(self.pos_diff)) < self.pos_thres and
        #        np.any(np.array(self.rot_diff)) < self.rot_thres):
        #    return False
        #else:
        #    return True
            

class LinearReactiveMovement:

    ##
    # @param arm 'l' or 'r'
    # @param pr2 Pr2 object (pr2.py)
    # @param tf_listener a tf.TransformListener()
    def __init__(self, arm, pr2, tf_listener, using_slip_controller=1, using_slip_detection=1):
        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener
        self.pr2 = pr2

        if arm == 'l':
            self.ik_frame = 'l_wrist_roll_link'
            self.tool_frame = 'l_gripper_tool_frame'
            self.arm_obj = self.pr2.left
            ptopic = '/pressure/l_gripper_motor'
        else:
            self.ik_frame = 'r_wrist_roll_link'
            self.tool_frame = 'r_gripper_tool_frame'
            self.arm_obj = self.pr2.right
            ptopic = '/pressure/r_gripper_motor'

        self.pressure_listener = pm.PressureListener(ptopic, 5000)

        self.cman = con.ControllerManager(arm, self.tf_listener, using_slip_controller,
                                          using_slip_detection)
        self.reactive_gr = rgr.ReactiveGrasper(self.cman)
        #self.collision_monitor = cmon.CollisionClient(arm)

        #cpy from kaijen code
        #gripper_event_detector_action_name = arm+'_gripper_sensor_controller/event_detector'
        #self.gripper_event_detector_action_client = actionlib.SimpleActionClient(gripper_event_detector_action_name, \
        #                                                                                     PR2GripperEventDetectorAction)

        self.movement_mode = 'cart' #or cart
        #self.cman.start_joint_controllers()
        #self.cman.start_gripper_controller()
        self.timeout = 50.

    ##
    # TODO is this redundant?
    # @return 3x1 pos matrix, and 4x1 orientation matrix both in base_link
    #def current_location(self):
    #    pos, rot = tfu.matrix_as_tf(self.arm_obj.pose_cartesian())
    #    return np.matrix(pos).T, np.matrix(rot).T

    ##
    # Moves to a given position, orientation
    #
    # @param loc (3x1 position matrix, 4x1 orientation matrix) in base_link
    # @param stop 'pressure_accel', 'pressure'
    # @param pressure pressure to use
    def move_absolute_old(self, loc, stop='pressure_accel', pressure=300):
        self.set_pressure_threshold(pressure)
        stop_funcs = self._process_stop_option(stop)
        self.set_movement_mode_cart()
        #pdb.set_trace()
        self._move_cartesian(loc[0], loc[1], stop_funcs, timeout=self.timeout, settling_time=5.0)
        self.set_movement_mode_ik()
        r = self._move_cartesian(loc[0], loc[1], stop_funcs, timeout=self.timeout, settling_time=5.0)
        #tfu.matrix_as_tf(self.arm_obj.pose_cartesian())[0]
        #diff = loc[0] - self.current_location()[0]
        diff = loc[0] - self.arm_obj.pose_cartesian_tf()[0]
        rospy.loginfo('move_absolute: diff is %s' % str(diff.T))
        rospy.loginfo('move_absolute: dist %.3f' % np.linalg.norm(diff))
        return r, np.linalg.norm(diff)


    def move_absolute(self, loc, stop='pressure_accel', pressure=300, frame='base_link'):
        self.set_pressure_threshold(pressure)
        stop_funcs = self._process_stop_option(stop)
        r = self._move_cartesian(loc[0], loc[1], stop_funcs, timeout=self.timeout,
                                 settling_time=5.0, frame=frame)
        diff = loc[0] - self.arm_obj.pose_cartesian_tf()[0]
        rospy.loginfo('move_absolute: diff is %s' % str(diff.T))
        rospy.loginfo('move_absolute: dist %.3f' % np.linalg.norm(diff))
        return r, np.linalg.norm(diff)

    ##
    # Moves relative to an arbitrary frame
    #
    # @param movement_target 3x1 matrix a displacement in the target frame
    # @param target_frame string id of the frame in which the movement is defined
    # @param stop 'pressure_accel', 'pressure'
    # @param pressure pressure to use
    def move_relative(self, movement_target, target_frame, stop='pressure_accel', pressure=150):
        base_T_target = tfu.transform('base_link', target_frame, self.tf_listener)
        movement_base = base_T_target[0:3, 0:3] * movement_target
        return self.move_relative_base(movement_base, stop=stop, pressure=pressure)

    ##
    # Moves relative to tool frame
    #
    # @param movement_tool 3x1 matrix a displacement in the tool frame
    # @param stop 'pressure_accel', 'pressure'
    # @param pressure pressure to use
    def move_relative_gripper(self, movement_tool, stop='pressure_accel', pressure=150):
        base_T_tool = tfu.transform('base_link', self.tool_frame, self.tf_listener)
        movement_base = base_T_tool[0:3, 0:3] * movement_tool # np.concatenate((movement_tool, np.matrix([1])))
        return self.move_relative_base(movement_base, stop=stop, pressure=pressure)

    ##
    # Moves relative to base frame
    #
    # @param movement 3x1 matrix displacement in base_frame
    # @param stop 'pressure_accel', 'pressure'
    # @param pressure pressure to use
    def move_relative_base(self, movement, stop='pressure_accel', pressure=150):
        self.set_pressure_threshold(pressure)
        trans, rot = self.arm_obj.pose_cartesian_tf()
        ntrans = trans + movement
        stop_funcs = self._process_stop_option(stop)
        r = self._move_cartesian(ntrans, rot, stop_funcs, \
                timeout=self.timeout, settling_time=5.0)


        diff = self.arm_obj.pose_cartesian_tf()[0] - (trans + movement)
        rospy.loginfo('move_relative_base: diff is ' + str(diff.T))
        rospy.loginfo('move_relative_base: dist %.3f' % np.linalg.norm(diff))
        return r, np.linalg.norm(diff)

    ##
    # Close gripper
    def gripper_close(self):
        self.reactive_gr.compliant_close()

    ##
    # Open gripper
    def gripper_open(self):
        self.reactive_gr.cm.command_gripper(.1, -1, 1)

    def set_pressure_threshold(self, t):
        self.pressure_listener.set_threshold(t)

    ##
    # Change which set of controllers are being used for move_* commands
    def set_movement_mode_ik(self):
        self.movement_mode = 'ik'
        self.reactive_gr.cm.switch_to_joint_mode()
        self.reactive_gr.cm.freeze_arm()

    ##
    # Change which set of controllers are being used for move_* commands
    def set_movement_mode_cart(self):
        self.movement_mode = 'cart'

    def _move_cartesian(self, position, orientation, \
            stop_funcs=[], timeout = 3.0, settling_time = 0.5, \
            frame='base_link', vel=.15):
        if self.movement_mode == 'ik':
            return self._move_cartesian_ik(position, orientation, stop_funcs, \
                    timeout, settling_time, frame, vel=.15)
        elif self.movement_mode == 'cart':
            return self._move_cartesian_cart(position, orientation, stop_funcs, \
                    timeout, settling_time, frame)


    ##
    # move the wrist to a desired Cartesian pose while watching the fingertip sensors
    # settling_time is how long to wait after the controllers think we're there
    def _move_cartesian_cart(self, position, orientation, \
            stop_funcs=[], timeout = 3.0, settling_time = 0.5, frame='base_link'):

        # TODO: Test this function...
        # Transform the pose from 'frame' to 'base_link'
        self.tf_listener.waitForTransform('base_link', frame, rospy.Time(),
                                          rospy.Duration(10))
        frame_T_base = tfu.transform('base_link', frame, self.tf_listener)
        init_cart_pose = tfu.tf_as_matrix((position.A1.tolist(),
                                           orientation.A1.tolist()))
        base_cart_pose = frame_T_base*init_cart_pose

        # Get IK from tool frame to wrist frame for control input
        self.tf_listener.waitForTransform(self.ik_frame, self.tool_frame, rospy.Time(), rospy.Duration(10))
        toolframe_T_ikframe = tfu.transform(self.tool_frame, self.ik_frame, self.tf_listener)
        #base_cart_pose = tfu.tf_as_matrix((base_position.A1.tolist(), base_orientation.A1.tolist()))
        base_cart_pose = base_cart_pose * toolframe_T_ikframe
        base_position, base_orientation = tfu.matrix_as_tf(base_cart_pose)

        pose_stamped = cf.create_pose_stamped(base_position.tolist() + base_orientation.tolist())
        rg = self.reactive_gr
        rg.check_preempt()

        #send the goal to the Cartesian controllers
        #rospy.loginfo("sending goal to Cartesian controllers")
        (pos, rot) = cf.pose_stamped_to_lists(rg.cm.tf_listener, pose_stamped, 'base_link')
        rg.move_cartesian_step(pos+rot, timeout, settling_time)

        #watch the fingertip/palm sensors until the controllers are done and then some
        start_time = rospy.get_rostime()
        done_time = None
        #stopped = False
        stop_trigger = None
        #print 'enterning loop'
        stop_detector = ArmStoppedDetector()
        while(1):

            rg.check_preempt()
            if len(stop_funcs) > 0:
                for f, name in stop_funcs:
                    if f():
                        rg.cm.switch_to_joint_mode()
                        rg.cm.freeze_arm()
                        #stopped = True
                        stop_trigger = name
                        rospy.loginfo('"%s" requested that motion should be stopped.' % (name))
                        break
                if stop_trigger != None:
                    break

            #if stop_func != None and stop_func():
            #    rg.cm.switch_to_joint_mode()
            #    rg.cm.freeze_arm()
            #    stopped = True
            #    break

            #check if we're actually there
            if rg.cm.check_cartesian_near_pose(pose_stamped, .0025, .05):
                rospy.loginfo("_move_cartesian_cart: reached pose")
                #stop_trigger = 'at_pose'
                break

            stop_detector.record_diff(self.arm_obj.pose_cartesian())
            if stop_detector.is_stopped():
                rospy.loginfo("_move_cartesian_cart: arm stopped")
                #stop_trigger = 'stopped'
                break

            # if rg.cm.check_cartesian_really_done(pose_stamped, .0025, .05):
            #     #rospy.loginfo("actually got there")
            #     break
            #
            # #check if the controllers think we're done
            # if not done_time and rg.cm.check_cartesian_done():
            #     #rospy.loginfo("check_cartesian_done returned 1")
            #     done_time = rospy.get_rostime()

            # #done settling
            # if done_time and rospy.get_rostime() - done_time > rospy.Duration(settling_time):
            #     rospy.loginfo("done settling")
            #     break

            #timed out
            #if timeout != 0. and rospy.get_rostime() - start_time > rospy.Duration(timeout):
            #    rospy.loginfo("_move_cartesian_cart: timed out")
            #    break

        #if stop_trigger == 'pressure_safety' or stop_trigger == 'self_collision':
        if stop_trigger == 'pressure_safety':
            print 'ROBOT SAFETY ERROR'
            #raise RobotSafetyError(stop_trigger)
        #name = ut.formatted_time() + '_stop_detector.pkl'
        #print 'saved', name
        #ut.save_pickle(stop_detector, name)
        return stop_trigger

    def _move_cartesian_ik(self, position, orientation, \
            stop_funcs=[], timeout = 30., settling_time = 0.25, \
            frame='base_link', vel=.15):
        #pdb.set_trace()
        #self.arm_obj.set_cart_pose_ik(cart_pose, total_time=motion_length, frame=frame, block=False)
        #cart_pose = tfu.tf_as_matrix((position.A1.tolist(), orientation.A1.tolist()))

        self.tf_listener.waitForTransform(self.ik_frame, self.tool_frame, rospy.Time(), rospy.Duration(10))
        toolframe_T_ikframe = tfu.transform(self.tool_frame, self.ik_frame, self.tf_listener)
        cart_pose = tfu.tf_as_matrix((position.A1.tolist(), orientation.A1.tolist()))
        cart_pose = cart_pose * toolframe_T_ikframe
        position, orientation = tfu.matrix_as_tf(cart_pose)

        #goal_pose_ps = create_pose_stamped(position.A1.tolist() + orientation.A1.tolist(), frame)
        goal_pose_ps = cf.create_pose_stamped(position.tolist() + orientation.tolist(), frame)
        r = self.reactive_gr.cm.move_cartesian_ik(goal_pose_ps, blocking=0, step_size=.005, \
                pos_thres=.02, rot_thres=.1, timeout=rospy.Duration(timeout),
                settling_time=rospy.Duration(settling_time), vel=vel)
        if not (r == 'sent goal' or r == 'success'):
            return r

        #move_cartesian_ik(self, goal_pose, collision_aware = 0, blocking = 1,                            
        #                  step_size = .005, pos_thres = .02, rot_thres = .1,                                
        #                  timeout = rospy.Duration(30.),                                                    
        #                  settling_time = rospy.Duration(0.25), vel = .15):   

        stop_trigger = None
        done_time = None
        start_time = rospy.get_rostime()
        while stop_trigger == None:
            for f, name in stop_funcs:
                if f():
                    self.arm_obj.stop_trajectory_execution()
                    stop_trigger = name
                    rospy.loginfo('"%s" requested that motion should be stopped.' % (name))
                    break

            if timeout != 0. and rospy.get_rostime() - start_time > rospy.Duration(timeout):
                rospy.loginfo("_move_cartesian_ik: motion timed out")
                break

            if (not done_time) and (not self.arm_obj.has_active_goal()):
                #rospy.loginfo("check_cartesian_done returned 1")
                done_time = rospy.get_rostime()

            if done_time and rospy.get_rostime() - done_time > rospy.Duration(settling_time):
                rospy.loginfo("_move_cartesian_ik: done settling")
                break

        if stop_trigger == 'pressure_safety':
            print 'ROBOT SAFETY ERROR'
            #raise RobotSafetyError(stop_trigger)
        return stop_trigger

    def _check_gripper_event(self):
        #state = self.gripper_event_detector_action_client.get_state()
        state = self.cman.get_gripper_event_detector_state()
        if state not in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
            rospy.loginfo('Gripper event detected.')
            return True 
        else:
            return False


    ##start up gripper event detector to detect when an object hits the table 
    #or when someone is trying to take an object from the robot
    def _start_gripper_event_detector(self, event_type = 'all', accel = 8.25, slip=.008, blocking = 0, timeout = 15.):
    
        goal = PR2GripperEventDetectorGoal()
        if event_type == 'accel':
            goal.command.trigger_conditions = goal.command.ACC
        elif event_type == 'slip':
            goal.command.trigger_conditions = goal.command.SLIP
        elif event_type == 'press_accel':
            goal.command.trigger_conditions = goal.command.FINGER_SIDE_IMPACT_OR_ACC
        elif event_type == 'slip_accel':
            goal.command.trigger_conditions = goal.command.SLIP_AND_ACC
        else:
            goal.command.trigger_conditions = goal.command.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC  #use either slip or acceleration as a contact condition

        #goal.command.trigger_conditions = goal.command.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC  #use either slip or acceleration as a contact condition
        goal.command.acceleration_trigger_magnitude = accel  #contact acceleration used to trigger 
        goal.command.slip_trigger_magnitude = slip           #contact slip used to trigger    
        
        rospy.loginfo("starting gripper event detector")
        self.cman.gripper_event_detector_action_client.send_goal(goal)
        
        #if blocking is requested, wait until the action returns
        if blocking:
            finished_within_time = self.cman.gripper_event_detector_action_client.wait_for_result(rospy.Duration(timeout))
            if not finished_within_time:
                rospy.logerr("Gripper didn't see the desired event trigger before timing out")
                return 0
            state = self.cman.gripper_event_detector_action_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                result = self.cman.gripper_event_detector_action_client.get_result()
                if result.data.placed:
                    return 1
            return 0

    def _process_stop_option(self, stop):
        stop_funcs = []
        self.pressure_listener.check_safety_threshold()
        #self.collision_monitor.check_self_contacts()

        #stop_funcs.append([self.pressure_listener.check_safety_threshold, 'pressure_safety'])
        #stop_funcs.append([self.collision_monitor.check_self_contacts, 'self_collision'])

        if stop == 'pressure':
            self.pressure_listener.check_threshold()
            stop_funcs.append([self.pressure_listener.check_threshold, 'pressure'])

        elif stop == 'pressure_accel':
            #print 'USING ACCELEROMETERS'
            #set a threshold for pressure & check for accelerometer readings
            self.pressure_listener.check_threshold()
            stop_funcs.append([self.pressure_listener.check_threshold, 'pressure'])

            self._start_gripper_event_detector(event_type='accel', timeout=self.timeout)
            stop_funcs.append([self._check_gripper_event, 'accel'])

        return stop_funcs




if __name__ == '__main__':
    mode = 'run'

    if mode == 'plot':
        import pylab as pb
        import sys
        
        a = ut.load_pickle(sys.argv[1])
        print len(a.pos_diff)
        pb.plot(a.pos_diff)
        pb.show()
        
        exit(0)

    else:

        import hrl_pr2_lib.pr2 as pr2
        rospy.init_node('test_linear_move')
        arm = 'r'
        tflistener = tf.TransformListener()
        robot = pr2.PR2(tflistener)
        movement = LinearReactiveMovement(arm, robot, tflistener)

        if mode == 'save':
            poses = []
            print 'going.....'
            while True:
                print 'waiting for input'
                r = raw_input()
                if r != 's':
                    print 'getting pose.'
                    p = movement.arm_obj.pose_cartesian()
                    print 'pose is', p
                    poses.append(p)
                else:
                    break

            ut.save_pickle(poses, 'saved_poses.pkl')

        elif mode == 'run':
            poses = ut.load_pickle('saved_poses.pkl')
            for p in poses:
                print 'hit enter to move'
                r = raw_input()
                pos, rot = tfu.matrix_as_tf(p)
                movement.set_movement_mode_cart()
                movement.move_absolute2((np.matrix(pos), np.matrix(rot)))
