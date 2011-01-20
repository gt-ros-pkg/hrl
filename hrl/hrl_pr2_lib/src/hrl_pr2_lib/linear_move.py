import roslib; roslib.load_manifest('hrl_pr2_lib')
import rospy

import pr2_gripper_reactive_approach.reactive_grasp as rgr 
import pr2_gripper_reactive_approach.controller_manager as con 
import object_manipulator.convert_functions as cf 

from actionlib_msgs.msg import GoalStatus 
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion 
from pr2_gripper_sensor_msgs.msg import PR2GripperEventDetectorGoal #TODO: manifest

import hai_sandbox.collision_monitor as cmon #TODO: move collision monitor here
#import hrl_pr2_lib.devices as de
import hrl_lib.tf_utils as tfu
import hrl_pr2_lib.pressure_listener as pm
import numpy as np
import tf
import pdb


class RobotSafetyError(Exception):
    def __init__(self, value):
        self.parameter = value

    def __str__(self):
        return repr(self.parameter)


class LinearReactiveMovement:

    ##
    # @param arm 'l' or 'r'
    # @param pr2 Pr2 object (pr2.py)
    # @param tf_listener a tf.TransformListener()
    def __init__(self, arm, pr2, tf_listener):
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

        self.pr2 = pr2
        self.pressure_listener = pm.PressureListener(ptopic, 5000)
        self.cman = con.ControllerManager(arm, self.tf_listener, using_slip_controller=1)
        self.reactive_gr = rgr.ReactiveGrasper(self.cman)
        self.collision_monitor = cmon.CollisionClient(arm)

        self.movement_mode = 'ik' #or cart
        self.cman.start_joint_controllers()
        self.cman.start_gripper_controller()
        self.timeout = 10.0

    ##
    # TODO is this redundant?
    # @return 3x1 pos matrix, and 4x1 orientation matrix both in base_link
    def current_location(self):
        pos, rot = tfu.matrix_as_tf(self.arm_obj.pose_cartesian())
        return np.matrix(pos).T, np.matrix(rot).T

    ##
    # Moves to a given position, orientation
    #
    # @param loc (3x1 position matrix, 4x1 orientation matrix) in base_link
    # @param stop 'pressure_accel', 'pressure'
    # @param pressure pressure to use
    def move_absolute(self, loc, stop='pressure_accel', pressure=300):
        self.set_pressure_threshold(pressure)
        stop_funcs = self._process_stop_option(stop)
        self.set_movement_mode_cart()
        #pdb.set_trace()
        self._move_cartesian(loc[0], loc[1], stop_funcs, timeout=self.timeout, settling_time=5.0)
        self.set_movement_mode_ik()
        r = self._move_cartesian(loc[0], loc[1], stop_funcs, timeout=self.timeout, settling_time=5.0)
        diff = loc[0] - self.current_location()[0]
        rospy.loginfo('move_absolute: diff is %s' % str(diff.T))
        rospy.loginfo('move_absolute: dist %.3f' % np.linalg.norm(diff))
        return r, np.linalg.norm(diff)

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
        trans, rot = self.current_location()
        ntrans = trans + movement
        stop_funcs = self._process_stop_option(stop)
        r = self._move_cartesian(ntrans, rot, stop_funcs, \
                timeout=self.timeout, settling_time=5.0)
        diff = self.current_location()[0] - (trans + movement)
        print 'move_relative_base: diff is ', diff.T
        print 'move_relative_base: dist %.3f' % np.linalg.norm(diff)
        return r

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
                    timeout, settling_time)

    ##
    # move the wrist to a desired Cartesian pose while watching the fingertip sensors
    # settling_time is how long to wait after the controllers think we're there
    def _move_cartesian_cart(self, position, orientation, \
            stop_funcs=[], timeout = 3.0, settling_time = 0.5):

        self.tf_listener.waitForTransform(self.ik_frame, self.tool_frame, rospy.Time(), rospy.Duration(10))
        toolframe_T_ikframe = tfu.transform(self.tool_frame, self.ik_frame, self.tf_listener)
        cart_pose = tfu.tf_as_matrix((position.A1.tolist(), orientation.A1.tolist()))
        cart_pose = cart_pose * toolframe_T_ikframe
        position, orientation = tfu.matrix_as_tf(cart_pose)

        #pose_stamped = cf.create_pose_stamped(position.T.A1.tolist() + orientation.T.A1.tolist())
        pose_stamped = cf.create_pose_stamped(position.tolist() + orientation.tolist())
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
            if rg.cm.check_cartesian_really_done(pose_stamped, .0025, .05):
                #rospy.loginfo("actually got there")
                break
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
            if timeout != 0. and rospy.get_rostime() - start_time > rospy.Duration(timeout):
                rospy.loginfo("timed out")
                break

        #if stop_trigger == 'pressure_safety' or stop_trigger == 'self_collision':
        if stop_trigger == 'pressure_safety':
            raise RobotSafetyError(stop_trigger)
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
            raise RobotSafetyError(stop_trigger)
        return stop_trigger

    def _check_gripper_event(self):

        state = self.cman.get_gripper_event_detector_state()
        if state not in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
            rospy.loginfo('Gripper event detected.')
            return True 
        else:
            return False


    ##start up gripper event detector to detect when an object hits the table 
    #or when someone is trying to take an object from the robot
    def _start_gripper_event_detector(self, event_type = 'all', accel = 5.25, slip=.008, blocking = 0, timeout = 15.):
    
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
        self.collision_monitor.check_self_contacts()

        stop_funcs.append([self.pressure_listener.check_safety_threshold, 'pressure_safety'])
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
