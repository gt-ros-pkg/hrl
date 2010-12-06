import roslib; roslib.load_manifest('hai_sandbox')
import hrl_pr2_lib.pr2 as pr2
#import hrl_pr2_lib.pr2_kinematics as pk
import pdb
import pr2_gripper_reactive_approach.reactive_grasp as rgr
import pr2_gripper_reactive_approach.controller_manager as con
import numpy as np
import rospy
import tf.transformations as tr
import object_manipulator.convert_functions as cf
import math
from actionlib_msgs.msg import GoalStatus
import copy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import tf
from pr2_gripper_sensor_msgs.msg import PR2GripperEventDetectorGoal
import pr2_msgs.msg as pm
import hai_sandbox.collision_monitor as cmon
import cv
import time
import subprocess as sb
import hai_sandbox.find_split as fs
import hrl_camera.ros_camera as rc
#import hrl_pr2_lib.devices as de
import hrl_pr2_lib.devices as hd
import hrl_lib.rutils as ru
import hrl_lib.tf_utils as tfu
import hrl_lib.util as ut
import hrl_lib.prob as pr


#from sound_play.msg import SoundRequest

class RobotSafetyError(Exception):
    def __init__(self, value):
        self.parameter = value

    def __str__(self):
        return repr(self.parameter)

class TaskError(Exception):
    def __init__(self, value):
        self.parameter = value

    def __str__(self):
        return repr(self.parameter)

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
                            }

        start_location = (np.matrix([0.3, 0.15, 0.9]).T, np.matrix([0., 0., 0., 0.1]))
        movement       = np.matrix([.4, 0, 0.]).T
        self.seed = [Action('linear_move', [start_location, movement, 'pressure']),
                     Action('linear_move', [Action('current_location', [])])]
        self.run(self.seed)


class LaserPointerClient:
    def __init__(self, target_frame='/base_link', tf_listener=None, robot=None):
        self.dclick_cbs = []
        self.point_cbs = []
        self.target_frame = target_frame
        self.laser_point_base = None
        self.robot = robot
        self.base_sound_path = (sb.Popen(["rospack", "find", "hai_sandbox"], stdout=sb.PIPE).communicate()[0]).strip()

        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

        rospy.Subscriber('cursor3d', PointStamped, self.laser_point_handler)
        self.double_click = rospy.Subscriber('mouse_left_double_click', String, self.double_click_cb)
        self.robot.sound.waveSound(self.base_sound_path + '/sounds/beep.wav').play()

    def transform_point(self, point_stamped):
        point_head = point_stamped.point
        #Tranform into base link
        base_T_head = tfu.transform(self.target_frame, point_stamped.header.frame_id, self.tf_listener)
        point_mat_head = tfu.translation_matrix([point_head.x, point_head.y, point_head.z])
        point_mat_base = base_T_head * point_mat_head
        t_base, _ = tfu.matrix_as_tf(point_mat_base)
        return np.matrix(t_base).T
        
    def laser_point_handler(self, point_stamped):
        self.robot.sound.waveSound(self.base_sound_path + '/sounds/blow.wav').play()
        self.laser_point_base = self.transform_point(point_stamped)
        for f in self.point_cbs:
            f(self.laser_point_base)

    def double_click_cb(self, a_str):
        rospy.loginfo('Double CLICKED')
        self.robot.sound.waveSound(self.base_sound_path + '/sounds/beep.wav').play()
        if self.laser_point_base != None:
            for f in self.dclick_cbs:
                f(self.laser_point_base)
            self.laser_point_base = None

    def add_double_click_cb(self, func):
        self.dclick_cbs.append(func)

    def add_point_cb(self, func):
        self.point_cbs.append(func)


class PressureListener:
    def __init__(self, topic='/pressure/l_gripper_motor', safe_pressure_threshold = 4000):
        rospy.Subscriber(topic, pm.PressureState, self.press_cb)
        self.lmat0 = None
        self.rmat0 = None

        self.safe_pressure_threshold = safe_pressure_threshold
        self.exceeded_safe_threshold = False

        self.threshold = None
        self.exceeded_threshold = False

    def check_safety_threshold(self):
        r = self.exceeded_safe_threshold
        self.exceeded_safe_threshold = False #reset 
        return r

    def check_threshold(self):
        r = self.exceeded_threshold
        self.exceeded_threshold = False
        return r

    def set_threshold(self, threshold):
        self.threshold = threshold

    def press_cb(self, pmsg):
        lmat = np.matrix((pmsg.l_finger_tip)).T
        rmat = np.matrix((pmsg.r_finger_tip)).T
        if self.lmat0 == None:
            self.lmat0 = lmat
            self.rmat0 = rmat
            return
    
        lmat = lmat - self.lmat0
        rmat = rmat - self.rmat0
       
        #touch detected
        if np.any(np.abs(lmat) > self.safe_pressure_threshold) or np.any(np.abs(rmat) > self.safe_pressure_threshold):
            self.exceeded_safe_threshold = True

        if self.threshold != None and (np.any(np.abs(lmat) > self.threshold) or np.any(np.abs(rmat) > self.threshold)):
            print 'EXCEEDED threshold', self.threshold
            self.exceeded_threshold = True


class Behaviors:
    def __init__(self, arm, tf_listener=None):
        try:
            rospy.init_node('linear_move', anonymous=True)
        except Exception, e:
            rospy.loginfo('call to init_node failed')

        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

        self.cman = con.ControllerManager(arm, self.tf_listener, using_slip_controller=1)
        self.reactive_gr = rgr.ReactiveGrasper(self.cman)
        if arm == 'l':
            self.tool_frame = 'l_gripper_tool_frame'
            ptopic = '/pressure/l_gripper_motor'
        else:
            self.tool_frame = 'r_gripper_tool_frame'
            ptopic = '/pressure/r_gripper_motor'
        self.pressure_listener = PressureListener(ptopic, 5000)
        self.collision_monitor = cmon.CollisionClient(arm)

        self.cman.start_joint_controllers()
        self.cman.start_gripper_controller()
        self.timeout = 10.0

        #rospy.Subscriber('cursor3d', PointStamped, self.laser_point_handler)
        #self.double_click = rospy.Subscriber('mouse_left_double_click', String, self.double_click_cb)

    def reach(self, point, pressure_thres, move_back_distance):
        self.set_pressure_threshold(pressure_thres)
        loc = self.current_location()[0]
        front_loc = point.copy()
        front_loc[0,0] = loc[0,0]

        start_loc = self.current_location()
        r1 = self.move_absolute((front_loc, start_loc[1]), stop='pressure_accel', pressure=pressure_thres)
        if r1 != None: #if this step fails, we move back then return
            #self.move_absolute(start_loc, stop='accel')
            return False, r1

        r2 = self.move_absolute((point, self.current_location()[1]), stop='pressure_accel', pressure=pressure_thres)
        touch_loc = self.current_location()
        if r2 == None or r2 == 'pressure' or r2 == 'accel':
            self.move_relative_gripper(move_back_distance, stop='none', pressure=pressure_thres)
            return True, r2, touch_loc
        else:
            #shouldn't get here
            return False, r2, None

    def press(self, direction, press_pressure, contact_pressure):
        #make contact first
        r1 = self.move_relative_gripper(direction, stop='pressure', pressure=contact_pressure)
        #now perform press
        if r1 == 'pressure' or r1 == 'accel':
            r2 = self.move_relative_gripper(direction, stop='pressure_accel', pressure=press_pressure)
            if r2 == 'pressure' or r2 == 'accel' or r2 == None:
                return True, r2
            else:
                return False, r2
        else:
            return False, r1

    def set_pressure_threshold(self, t):
        self.pressure_listener.set_threshold(t)

    def move_absolute(self, loc, stop='pressure_accel', pressure=300):
        self.set_pressure_threshold(pressure)
        stop_funcs = self._process_stop_option(stop)
        return self._move_cartesian(loc[0], loc[1], stop_funcs, timeout=self.timeout, settling_time=5.0)

    def move_relative_base(self, movement, stop='pressure_accel', pressure=150):
        self.set_pressure_threshold(pressure)
        trans, rot = self.current_location()
        ntrans = trans + movement
        stop_funcs = self._process_stop_option(stop)
        return self._move_cartesian(ntrans, rot, stop_funcs, timeout=self.timeout, settling_time=5.0)

    def move_relative_gripper(self, movement_tool, stop='pressure_accel', pressure=150):
        #pdb.set_trace()
        base_T_tool = tfu.transform('base_link', self.tool_frame, self.tf_listener)
        movement_base = base_T_tool[0:3, 0:3] * movement_tool# np.concatenate((movement_tool, np.matrix([1])))
        #pdb.set_trace()
        return self.move_relative_base(movement_base, stop=stop, pressure=pressure)

    def twist(self, angle):
        pos, rot = self.cman.return_cartesian_pose() # in base_link
        ax, ay, az = tr.euler_from_quaternion(rot) 
        nrot = tr.quaternion_from_euler(ax + angle, ay, az)
        #pdb.set_trace()
        stop_funcs = self._process_stop_option(stop)
        return self._move_cartesian(np.matrix(pos).T, np.matrix(nrot).T, stop_funcs, timeout=self.timeout, settling_time=5.0)

    def gripper_close(self):
        self.reactive_gr.compliant_close()

    def gripper_open(self):
        self.reactive_gr.cm.command_gripper(.1, -1, 1)

    def current_location(self):
        pos, rot = self.cman.return_cartesian_pose()
        return np.matrix(pos).T, np.matrix(rot).T

    def _process_stop_option(self, stop):
        stop_funcs = []
        self.pressure_listener.check_safety_threshold()
        self.collision_monitor.check_self_contacts()

        stop_funcs.append([self.pressure_listener.check_safety_threshold, 'pressure_safety'])
        stop_funcs.append([self.collision_monitor.check_self_contacts, 'self_collision'])

        if stop == 'pressure':
            self.pressure_listener.check_threshold()
            stop_funcs.append([self.pressure_listener.check_threshold, 'pressure'])

        elif stop == 'pressure_accel':
            #print 'USING ACCELEROMETERS'
            #set a threshold for pressure & check for accelerometer readings
            self.pressure_listener.check_threshold()
            stop_funcs.append([self.pressure_listener.check_threshold, 'pressure'])

            self.start_gripper_event_detector(event_type='accel', timeout=self.timeout)
            stop_funcs.append([self._check_gripper_event, 'accel'])

        return stop_funcs

    ##start up gripper event detector to detect when an object hits the table 
    #or when someone is trying to take an object from the robot
    def start_gripper_event_detector(self, event_type = 'all', accel = 3.25, slip=.008, blocking = 0, timeout = 15.):
    
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


    def _check_gripper_event(self):
        #r1 = self.pressure_listener.check_threshold() 
        #r2 = self.pressure_listener.check_safety_threshold()
        #if r1:
        #    rospy.loginfo('Pressure exceeded!')
        #if r2:
        #    rospy.loginfo('Pressure safety limit EXCEEDED!')
        #pressure_state = r1 or r2
        #pressure_state = self.pressure_listener.check_threshold() or self.pressure_listener.check_safety_threshold()
        #action finished (trigger seen)

        state = self.cman.get_gripper_event_detector_state()
        if state not in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
            rospy.loginfo('Gripper event detected.')
            return True 
        else:
            return False

    #def _tactile_stop_func(self):
    #    r1 = self.pressure_listener.check_threshold() 
    #    r2 = self.pressure_listener.check_safety_threshold()
    #    if r1:
    #        rospy.loginfo('Pressure exceeded!')
    #    if r2:
    #        rospy.loginfo('Pressure safety limit EXCEEDED!')
    #    return r1 or r2


    ##move the wrist to a desired Cartesian pose while watching the fingertip sensors
    #settling_time is how long to wait after the controllers think we're there
    def _move_cartesian(self, position, orienation, \
            stop_funcs=[], timeout = 3.0, settling_time = 0.5):
        pose_stamped = cf.create_pose_stamped(position.T.A1.tolist() + orienation.T.A1.tolist())
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

            #check if the controllers think we're done
            if not done_time and rg.cm.check_cartesian_done():
                #rospy.loginfo("check_cartesian_done returned 1")
                done_time = rospy.get_rostime()

            #done settling
            if done_time and rospy.get_rostime() - done_time > rospy.Duration(settling_time):
                #rospy.loginfo("done settling")
                break

            #timed out
            if timeout != 0. and rospy.get_rostime() - start_time > rospy.Duration(timeout):
                #rospy.loginfo("timed out")
                break

        if stop_trigger == 'pressure_safety' or stop_trigger == 'self_collision':
            raise RobotSafetyError(stop_trigger)
        return stop_trigger


def image_diff_val2(before_frame, after_frame):
    br = np.asarray(before_frame)
    ar = np.asarray(after_frame)
    max_sum = br.shape[0] * br.shape[1] * br.shape[2] * 255.
    sdiff = np.abs((np.sum(br) / max_sum) - (np.sum(ar) / max_sum))
    #sdiff = np.sum(np.abs(ar - br)) / max_sum
    return sdiff


class BehaviorTest:

    def __init__(self):
        rospy.init_node('linear_move', anonymous=True)
        self.tf_listener = tf.TransformListener()
        self.behaviors = Behaviors('l', tf_listener=self.tf_listener)
        self.robot = pr2.PR2(self.tf_listener)
        self.laser_scan = hd.LaserScanner('point_cloud_srv')
        #self.prosilica = rc.Prosilica('prosilica', 'streaming')
        self.prosilica = rc.Prosilica('prosilica', 'polled')
        self.prosilica_cal = rc.ROSCameraCalibration('/prosilica/camera_info')
        self.left_cal = rc.ROSCameraCalibration('/wide_stereo/left/camera_info')
        self.right_cal = rc.ROSCameraCalibration('/wide_stereo/right/camera_info')

        self.wide_angle_camera_left = rc.ROSCamera('/wide_stereo/left/image_rect_color')
        self.wide_angle_camera_right = rc.ROSCamera('/wide_stereo/right/image_rect_color')

        #pdb.set_trace()
        self.laser_listener = LaserPointerClient(tf_listener=self.tf_listener, robot=self.robot)
        self.laser_listener.add_double_click_cb(self.click_cb)

        #self.behaviors.set_pressure_threshold(300)
        self.start_location = (np.matrix([0.25, 0.10, 1.3]).T, np.matrix([0., 0., 0., 0.1]))

        self.critical_error = False


    def camera_change_detect(self, threshold, f, args):
        #take before sensor snapshot
        start_pose = self.robot.head.pose()
        self.robot.head.set_pose(np.radians(np.matrix([1.04, -20]).T), 1)
        time.sleep(4)
        for i in range(4):
            before_frame = self.wide_angle_camera_left.get_frame()
        cv.SaveImage('before.png', before_frame)
        f_return = f(*args)
        time.sleep(2)
        for i in range(3):
            after_frame = self.wide_angle_camera_left.get_frame()

        cv.SaveImage('after.png', after_frame)
        sdiff = image_diff_val2(before_frame, after_frame)
        #pdb.set_trace()
        self.robot.head.set_pose(start_pose, 1)
        time.sleep(3)        
        #take after snapshot
        #threshold = .03
        rospy.loginfo('camera difference %.4f (thres %.3f)' % (sdiff, threshold))
        if sdiff > threshold:
            rospy.loginfo('difference detected!')
            return True, f_return
        else:
            rospy.loginfo('NO differences detected!')
            return False, f_return


    def light_switch1(self, point, 
            point_offset, press_contact_pressure, move_back_distance,
            press_pressure, press_distance, visual_change_thres):
        print '===================================================================='
        point = point + point_offset 
        rospy.loginfo('REACHING')
        #self.behaviors.gripper_close()
        self.behaviors.move_absolute(self.start_location, stop='pressure_accel')

        #start_loc = self.current_location()
        success, reason, touchloc = self.behaviors.reach(point, press_contact_pressure, move_back_distance)
        if not success:
            error_msg = 'Reach failed due to "%s"' % reason
            rospy.loginfo(error_msg)
            rospy.loginfo('Failure recovery: moving back')
            self.behaviors.move_absolute(self.start_location, stop='accel', \
                    pressure=press_contact_pressure)
            #raise TaskError(error_msg)
            return False, None

        rospy.loginfo('PRESSING')
        change, press_ret = self.camera_change_detect(visual_change_thres, self.behaviors.press, \
                (press_distance, press_pressure, press_contact_pressure))
        success, reason = press_ret
        if not success:
            rospy.loginfo('Press failed due to "%s"' % reason)

        #code reward function
        #monitor self collision => collisions with the environment are not self collisions
        rospy.loginfo('MOVING BACK')
        r1 = self.behaviors.move_relative_gripper(np.matrix([-.03, 0., 0.]).T, \
                stop='none', pressure=press_contact_pressure)
        if r1 != None:
            rospy.loginfo('moving back failed due to "%s"' % r1)
            return False, None

        rospy.loginfo('RESETING')
        r2 = self.behaviors.move_absolute(self.start_location, stop='pressure_accel')
        if r2 != None:
            rospy.loginfo('moving back to start location failed due to "%s"' % r2)
            return False, None

        rospy.loginfo('DONE.')
        return change, touchloc


    def light_switch2(self, point):
        success, reason, touchloc = self.behaviors.reach(point)
        if not success:
            rospy.loginfo('Reach failed due to "%s"' % reason)

        rospy.loginfo('RESETING')
        r2 = self.behaviors.move_absolute(self.start_location, stop='pressure_accel')
        if r2 != None:
            rospy.loginfo('moving back to start location failed due to "%s"' % r2)
            return 

    #def optimize_parameters(self, x0, x_range, behavior, objective_func, reset_env_func, reset_param):
    #    reset_retries = 3
    #    num_params = len(x0)
    #    x = copy.deepcopy(x0)

    #    # for each parameter
    #    #for i in range(num_params):
    #    while i < num_params:
    #        #search for a good setting
    #        not_converged = True
    #        xmin = x_range[i, 0]
    #        xmax = x_range[i, 1]

    #        while not_converged:
    #            current_val = x[i]
    #            candidates_i = [(x[i] + xmin) / 2., (x[i] + xmax) / 2.]
    #            successes = []
    #            for cand in candidates_i:
    #                x[i] = cand
    #                success = behavior(x)
    #                if success:
    #                    for reset_i in range(reset_retries):
    #                        reset_success = reset_env_func(*reset_param)
    #                        if reset_success:
    #                            break
    #                successes.append(success)

    #            if successes[0] and successes[1]:
    #                raise RuntimeException('What? this isn\'t suppose to happen.')
    #            elif successes[0] and not successes[1]:
    #                next_val = candidates_i[0]
    #            elif successes[1] and not successes[0]:
    #                next_val = candidates_i[1]
    #            else:
    #                raise RuntimeException('What? this isn\'t suppose to happen.')


    #        #if all the trials are bad
    #        if not test(successes):
    #            #go back by 1 parameter
    #            i = i - 1


    #        #if there are more than one good parameter
    #        for p in params
    #            ... = objective_func(p)

    #        i = i + 1

    #    return x


    def record_perceptual_data(self, point_touched):
        #what position should the robot be in?
        #set arms to non-occluding pose

        #record region around the finger where you touched
        rospy.loginfo('Getting laser scan.')
        points = self.laser_scan.scan(math.radians(180.), math.radians(-180.), 10.)
        rospy.loginfo('Getting Prosilica image.')
        prosilica_image = self.prosilica.get_frame()
        rospy.loginfo('Getting image from left wide angle camera.')
        left_image  = self.wide_angle_camera_left.get_frame()
        rospy.loginfo('Getting image from right wide angle camera.')
        right_image = self.wide_angle_camera_left.get_frame()
        rospy.loginfo('Waiting for calibration.')
        while self.prosilica_cal.has_msg == False:
            time.sleep(.1)

        #which frames?
        rospy.loginfo('Getting transforms.')
        pro_T_bf = tfu.transform('/high_def_optical_frame', '/base_footprint', self.tf_listener)
        laser_T_bf = tfu.transform('/laser_tilt_link', '/base_footprint', self.tf_listener)
        tstring = time.strftime('%A_%m_%d_%Y_%I:%M%p')
        prosilica_name = '%s_highres.png' % tstring
        left_name = '%s_left.png' % tstring
        right_name = '%s_right.png' % tstring
        rospy.loginfo('Saving images (basename %s)' % tstring)
        cv.SaveImage(prosilica_name, prosilica_image)
        cv.SaveImage(left_name, left_image)
        cv.SaveImage(right_name, right_image)

        rospy.loginfo('Saving pickles')
        pickle_fname = '%s_interest_point_dataset.pkl' % tstring   
        ut.save_pickle({'touch_point': point_touched,
                        'points_laser': points,

                        'high_res': prosilica_name,
                        'left_image': left_name,
                        'right_image': right_name,

                        'laser_T_bf': laser_T_bf, 
                        'pro_T_bf': pro_T_bf,
                        'point_touched': point_touched,
                        
                        'prosilica_cal': self.prosilica_cal, 
                        'left_cal': self.left_cal,
                        'right_cal': self.right_cal},
                        pickle_fname)
        print 'Recorded to', pickle_fname


    def gather_interest_point_dataset(self, point):
        gaussian = pr.Gaussian(np.matrix([0, 0, 0.]).T, np.matrix([[1., 0, 0], [0, .02**2, 0], [0, 0, .02**2]]))

        for i in range(100):
            # perturb_point
            gaussian_noise = gaussian.sample()
            gaussian_noise[0,0] = 0
            npoint = point + gaussian_noise
            success_off, touchloc = self.light_switch1(npoint, 
                            point_offset=np.matrix([-.15,0,0]).T, press_contact_pressure=300, 
                            move_back_distance=np.matrix([-.005,0,0]).T, press_pressure=2500, 
                            press_distance=np.matrix([0,0,-.15]).T, visual_change_thres=.03)

            rospy.loginfo('Lights turned off? %s' % str(success_off))
            if success_off:
                self.record_perceptual_data(touchloc)
            
            #Turn on lights
            success_on, touchloc = self.light_switch1(npoint, 
                            point_offset=np.matrix([-.15,0,-.10]).T, press_contact_pressure=300, 
                            move_back_distance=np.matrix([-0.005, 0, 0]).T, press_pressure=2500, 
                            press_distance=np.matrix([0,0,.1]).T, visual_change_thres=.03)



    def click_cb(self, point):
        if self.critical_error:
            rospy.loginfo('Behaviors supressed due to uncleared critical error.')
            return

        try:
            print 'CLICKED on point', point.T
            self.gather_interest_point_dataset(point)
            #point = np.matrix([ 0.60956734, -0.00714498,  1.22718197]).T
            #pressure_parameters = range(1900, 2050, 30)

            #self.record_perceptual_data(point)
            #successes = []
            #parameters = [np.matrix([-.15, 0, 0]).T, 300, np.matrix([-.005, 0, 0]).T, 3500, np.matrix([0,0,-.15]).T, .03]

            #for p in pressure_parameters:
            #    experiment = []
            #    for i in range(4):
            #        #Turn off lights
            #        rospy.loginfo('Experimenting with press_pressure = %d' % p)
            #        success_off = self.light_switch1(point, 
            #                        point_offset=np.matrix([-.15,0,0]).T, press_contact_pressure=300, move_back_distance=np.matrix([-.005,0,0]).T,\
            #                        press_pressure=3500, press_distance=np.matrix([0,0,-.15]).T, visual_change_thres=.03)
            #        experiment.append(success_off)
            #        rospy.loginfo('Lights turned off? %s' % str(success_off))
            #        return

            #        #Turn on lights
            #        success_on = self.light_switch1(point, 
            #                        point_offset=np.matrix([-.15,0,-.10]).T, press_contact_pressure=300, move_back_distance=np.matrix([-0.005, 0, 0]).T,
            #                        press_pressure=3500, press_distance=np.matrix([0,0,.1]).T, visual_change_thres=.03)
            #        #def light_switch1(self, point, 
            #        #        point_offset, press_contact_pressure, move_back_distance,
            #        #        press_pressure, press_distance, visual_change_thres):

            #        print 'Lights turned on?', success_on
            #    successes.append(experiment)

            #ut.save_pickle({'pressure': pressure_parameters, 
            #                'successes': successes}, 'pressure_variation_results.pkl')

        except RobotSafetyError, e:
            rospy.loginfo('Caught a robot safety exception "%s"' % str(e.parameter))
            self.behaviors.move_absolute(self.start_location, stop='accel')

        except TaskError, e:
            rospy.loginfo('TaskError: %s' % str(e.parameter))


    def run(self):
        #point = np.matrix([ 0.60956734, -0.00714498,  1.22718197]).T
        #print 'RECORDING'
        #self.record_perceptual_data(point)
        #print 'DONE RECORDING'
        r = rospy.Rate(10)
        rospy.loginfo('Ready.')
        while not rospy.is_shutdown():
            r.sleep()


if __name__ == '__main__':
    l = BehaviorTest()
    l.run()











































        #return self.pressure_listener.check_threshold() or self.pressure_listener.check_safety_threshold()
        ##stop if you hit a tip, side, back, or palm
        #(left_touching, right_touching, palm_touching) = self.reactive_gr.check_guarded_move_contacts()
        ##saw a contact, freeze the arm
        #if left_touching or right_touching or palm_touching:
        #    rospy.loginfo("CONTACT made!")
        #    return True
        #else:
        #    return False

        #print 'move returning'
        #return whether the left and right fingers were touching
        #return (left_touching, right_touching, palm_touching)




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

        #point = np.matrix([0.63125642, -0.02918334, 1.2303758 ]).T
        #print 'move direction', movement.T
        #print 'CORRECTING', point.T
        #print 'NEW', point.T
        #start_location = (np.matrix([0.25, 0.15, 0.7]).T, np.matrix([0., 0., 0., 0.1]))
        #movement = np.matrix([.4, 0., 0.]).T
        #what other behavior would I want?
        # touch then move away..
        # move back but more slowly..
        # want a safe physical
        #   a safe exploration strategy
        #self.behaviors.linear_move(self.behaviors.current_location(), back_alittle, stop='none')
        #loc_before = self.behaviors.current_location()[0]
        #loc_after = self.behaviors.current_location()[0]
        #pdb.set_trace()
        #self.behaviors.linear_move(self.behaviors.current_location(), down, stop='pressure_accel')
        #self.behaviors.linear_move(self.behaviors.current_location(), back, stop='none')
        #pdb.set_trace()
        #b.twist(math.radians(30.))
        #bd = BehaviorDescriptor()
        #movement = point - self.behaviors.current_location()[0]
        #pdb.set_trace()
        #self.behaviors.linear_move(self.behaviors.current_location(), movement, stop='pressure_accel')

        #loc = self.behaviors.current_location()[0]
        #front_loc = point.copy()
        #front_loc[0,0] = loc[0,0]
        #self.behaviors.set_pressure_threshold(150)
        #self.behaviors.move_absolute((front_loc, self.behaviors.current_location()[1]), stop='pressure_accel')
        #self.behaviors.move_absolute((point, self.behaviors.current_location()[1]), stop='pressure_accel')




    #def detect_event(self):
    #    self.behaviors.cman.start_gripper_event_detector(timeout=40.)
    #    stop_func = self.behaviors._tactile_stop_func
    #    while stop_func():

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

    #def light_switch1_on(self, point, press_pressure=3500, press_contact_pressure=150):
    #    point = point + np.matrix([-.15, 0, -0.20]).T

    #    success, reason = self.behaviors.reach(point)
    #    if not success:
    #        rospy.loginfo('Reach failed due to "%s"' % reason)

    #    rospy.loginfo('PRESSING')
    #    success, reason = self.behaviors.press(np.matrix([0, 0, .20]).T, \
    #            press_pressure, press_contact_pressure)
    #    if not success:
    #        rospy.loginfo('Press failed due to "%s"' % reason)
    #        return 

    #    rospy.loginfo('RESETING')
    #    r2 = self.behaviors.move_absolute(self.start_location, stop='pressure_accel')
    #    if r2 != None:
    #        rospy.loginfo('moving back to start location failed due to "%s"' % r2)
    #        return 

    #    print 'DONE.'

