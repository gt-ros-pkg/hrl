# Extension and modification of code originally written by Kaijen Hsiao

#! /usr/bin/python
import numpy as np, math

import roslib; roslib.load_manifest('hrl_pr2_lib')
import rospy


from interpolated_ik_motion_planner.ik_utilities import *
from pr2_gripper_reactive_approach.controller_manager import *

class HRLIKUtilities(IKUtilities):

    #initialize all service functions
    #if wait_for_services = 0, you must call check_services_and_get_ik_info externally before running any of the IK/FK functions
    def __init__(self, whicharm, tf_listener = None, wait_for_services = 1): #whicharm is 'right' or 'left'
        self.whicharm = whicharm # TODO CHANGE
        
        #gets the robot_prefix from the parameter server. Default is pr2 
        robot_prefix = rospy.get_param('~robot_prefix', 'pr2') 
        self.srvroot = '/'+robot_prefix+'_'+whicharm+'_arm_kinematics/' 

        #If collision_aware_ik is set to 0, then collision-aware IK is disabled 
 	self.perception_running = rospy.get_param('~collision_aware_ik', 1) 

        self._ik_service = rospy.ServiceProxy(self.srvroot+'get_ik', GetPositionIK, True)
        if self.perception_running:
            self._ik_service_with_collision = rospy.ServiceProxy(self.srvroot+'get_constraint_aware_ik', GetConstraintAwarePositionIK, True)

        self._fk_service = rospy.ServiceProxy(self.srvroot+'get_fk', GetPositionFK, True
        self._query_service = rospy.ServiceProxy(self.srvroot+'get_ik_solver_info', GetKinematicSolverInfo, True)
        self._check_state_validity_service = rospy.ServiceProxy('/planning_scene_validity_server/get_state_validity', GetStateValidity, True)

        #wait for IK/FK/query services and get the joint names and limits 
        if wait_for_services:
            self.check_services_and_get_ik_info()
        
        if tf_listener == None:
            rospy.loginfo("ik_utilities: starting up tf_listener")
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

        self.marker_pub = rospy.Publisher('interpolation_markers', Marker)

        #dictionary for the possible kinematics error codes
        self.error_code_dict = {}  #codes are things like SUCCESS, NO_IK_SOLUTION
        for element in dir(ArmNavigationErrorCodes):
            if element[0].isupper():
                self.error_code_dict[eval('ArmNavigationErrorCodes.'+element)] = element

        #reads the start angles from the parameter server 
        start_angles_list = rospy.get_param('~ik_start_angles', []) 
 		         
        #good additional start angles to try for IK for the PR2, used  
        #if no start angles were provided 
        if start_angles_list == []: 
            self.start_angles_list = [[-0.697, 1.002, 0.021, -0.574, 0.286, -0.095, 1.699], 
                                      [-1.027, 0.996, 0.034, -0.333, -3.541, -0.892, 1.694], 
                                      [0.031, -0.124, -2.105, -1.145, -1.227, -1.191, 2.690], 
                                      [0.410, 0.319, -2.231, -0.839, -2.751, -1.763, 5.494], 
                                      [0.045, 0.859, 0.059, -0.781, -1.579, -0.891, 7.707], 
                                      [0.420, 0.759, 0.014, -1.099, -3.204, -1.907, 8.753], 
                                      [-0.504, 1.297, -1.857, -1.553, -4.453, -1.308, 9.572]] 
        else: 
            self.start_angles_list = start_angles_list 

        if whicharm == 'left':
            for i in range(len(self.start_angles_list)):
                for joint_ind in [0, 2, 4]:
                    self.start_angles_list[i][joint_ind] *= -1.

        #changes the set of ids used to show the arrows every other call
        self.pose_id_set = 0

        rospy.loginfo("ik_utilities: done init")

    #check a Cartesian path for consistent, non-colliding IK solutions
    #start_pose and end_pose are PoseStamped messages with the wrist poses
    #start_angles are angles to try to stay close to
    #num_steps is the number of interpolation steps to use (if 0, use 
    #  pos_spacing and rot_spacing instead to calculate the number of steps)
    #pos_spacing is max wrist translation in meters between trajectory points 
    #rot_spacing is max wrist rotation in radians between trajectory points
    #consistent_angle is the max joint angle change before 2 steps are declared inconsistent
    #collision_check_resolution is the resolution at which to check collisions (0 or 1 for every)
    #steps_before_abort is the number of invalid steps found before aborting (-1 to ignore)
    #if collision_aware is 0, ignore collisions
    #ordered_collision_operations is an optional list of collision operations to feed to IK to modify the collision space
    #link_padding is an optional list of link paddings to feed to IK to modify the robot collision padding
    #IK_robot_state is an optional RobotState message to pass to IK
    #if start_from_end is 1, find an IK solution for the end first and work backwards
    #returns the joint angle trajectory and the error codes (0=good, 
    #  1=collisions, 2=inconsistent, 3=out of reach, 4=aborted before checking)
    # Added joints_bias and bias_radius which define a joint configuration all IK
    # should prefer during the trajectory and bias_radius defines the rate at which
    # to move in that direction
    def check_cartesian_path(self, start_pose, end_pose, start_angles, pos_spacing = 0.01, rot_spacing = 0.1, consistent_angle = math.pi/9., collision_aware = 1, collision_check_resolution = 1, steps_before_abort = -1, num_steps = 0, ordered_collision_operations = None, start_from_end = 0, IK_robot_state = None, link_padding = None, use_additional_start_angles = 0, joints_bias = None, bias_radius = 0.0):
        if joints_bias is None: # CHANGE
            joints_bias = [0.0] * 7 # CHANGE
        
        start_angles = self.bias_guess(start_angles, joints_bias, bias_radius) # CHANGE

        #sanity-checking
        if num_steps != 0 and num_steps < 2:
            num_steps = 2
        if collision_check_resolution < 1:
            collision_check_resolution = 1
        if num_steps == 0 and (pos_spacing <= 0 or rot_spacing <= 0):
            rospy.logerr("invalid pos_spacing or rot_spacing")
            return ([], [])

        #convert to lists
        (start_pos, start_rot) = self.pose_stamped_to_lists(start_pose, 'base_link')
        (end_pos, end_rot) = self.pose_stamped_to_lists(end_pose, 'base_link')
        if start_pos == None or end_pos == None:
            return (None, None)
        
        #interpolate path
        steps = self.interpolate_cartesian(start_pos, start_rot, end_pos, end_rot, pos_spacing, rot_spacing, num_steps)

        #run collision-aware ik on each step, starting from the end and going backwards if start_from_end is true
        if start_from_end:
            steps.reverse()

        #use additional start angles from a pre-chosen set
        if use_additional_start_angles:
            num_to_use = max(use_additional_start_angles, len(self.start_angles_list))
            start_angles_list = [start_angles,] + self.start_angles_list[0:num_to_use]
        else:
            start_angles_list = [start_angles,]

        #go through each set of start angles, see if we can find a consistent trajectory
        for (start_angles_ind, start_angles) in enumerate(start_angles_list):
            trajectory = []
            error_codes = [] 
            
            if use_additional_start_angles:
                rospy.loginfo("start_angles_ind: %d"%start_angles_ind)

            for stepind in range(len(steps)):
                (pos,rot) = steps[stepind]
                pose_stamped = self.lists_to_pose_stamped(pos, rot, 'base_link', 'base_link')

                #draw the pose in rviz that we're checking in IK
                self.draw_pose(pose_stamped, stepind*3+self.pose_id_set*50)
                self.pose_id_set = (self.pose_id_set+1)%2

                #check for a non-collision_aware IK solution first
                (colliding_solution, error_code) = self.run_ik(pose_stamped, start_angles, self.link_name, collision_aware = 0, IK_robot_state = IK_robot_state)
                if not colliding_solution:
                    rospy.loginfo("non-collision-aware IK solution not found for step %d!"%stepind)
                    trajectory.append([0.]*7)
                    error_codes.append(3)          #3=out of reach

                else:
                    #if we're checking for collisions, then look for a collision-aware solution
                    collision_aware_this_step = collision_aware and (stepind % collision_check_resolution == 0 or stepind == len(steps)-1)
                    if collision_aware_this_step:
                        (solution, error_code) = self.run_ik(pose_stamped, start_angles, self.link_name, collision_aware, ordered_collision_operations, IK_robot_state = IK_robot_state, link_padding = link_padding)
                        if not solution:
                            rospy.loginfo("non-colliding IK solution not found for step %d!"%stepind)
                            collision_aware_solution_found = 0
                            solution = colliding_solution
                        else:
                            collision_aware_solution_found = 1
                    else:                
                        solution = colliding_solution

                    trajectory.append(list(solution))

                    #first trajectory point, or last point was all 0s, or consistent with previous point
                    if stepind == 0 or error_codes[-1] == 3 or self.check_consistent(trajectory[-2], solution, consistent_angle):
                        if not collision_aware_this_step or collision_aware_solution_found:
                            error_codes.append(0)  #0=good
                        else:
                            error_codes.append(1)  #1=collisions
                    else:
                        rospy.loginfo("IK solution not consistent for step %d!"%stepind)
                        error_codes.append(2)      #2=inconsistent

                    start_angles = solution
                    start_angles = self.bias_guess(start_angles, joints_bias, bias_radius) #CHANGE

                #check if we should abort due to finding too many invalid points
                if error_codes[-1] > 0 and steps_before_abort >= 0 and stepind >= steps_before_abort:
                    rospy.loginfo("aborting due to too many invalid steps")
                    trajectory.extend([[0.]*7 for i in range(len(steps)-stepind-1)])
                    error_codes.extend([4]*(len(steps)-stepind-1)) #4=aborted before checking
                    break

            #if we found a successful trajectory, stop and return it
            if not any(error_codes):
                break

        if start_from_end:
            trajectory.reverse()        
            error_codes.reverse()
        return (trajectory, error_codes)

    ##
    # Added function which biases the given angles towards a given configuration
    def bias_guess(self, q, joints_bias, bias_radius):
        if bias_radius == 0.0:
            return q
        if self.whicharm == 'right':
            max_angs = np.array([.69, 1.33, 0.79, 0.0, 1000000.0, 0.0, 1000000.0])
            min_angs = np.array([-2.27, -.54, -3.9, -2.34, -1000000.0, -2.15, -1000000.0])
        else:
            max_angs = np.array([2.27, 1.33, 3.9, 0.0, 1000000.0, 0.0, 1000000.0])
            min_angs = np.array([-.69, -.54, -0.79, -2.34, -1000000.0, -2.15, -1000000.0])
        q_off = bias_radius * np.array(joints_bias) / np.linalg.norm(joints_bias)
        angs = np.array(q) + q_off
        for i in range(7):
            if angs[i] > max_angs[i]:
                angs[i] = max_angs[i]
            elif angs[i] < min_angs[i]:
                angs[i] = min_angs[i]
        return angs.tolist()

    ##
    # runs ik but tries to bias it towards the given bias configuration
    def run_biased_ik(self, pose_stamped, joints_bias = [0.]*7, 
                      num_iters=6, init_angs=[0.]*7):
        angs = init_angs
        has_solution = False
        for i in range(num_iters):
            (solution, error_code) = self.run_ik(pose_stamped, angs, \
                                                self.link_name)
            if solution:
                angs = solution
                has_solution = True
            if i < num_iters - 1:
                angs = self.bias_guess(angs, joints_bias, 0.1)
        if has_solution:
            return angs
        else:
            return None


class HRLControllerManager(ControllerManager):
    #whicharm is 'r' or 'l'
    def __init__(self, whicharm, tf_listener = None, using_slip_controller = 0, using_slip_detection = 0): 
        self.whicharm = whicharm
  
        self.using_slip_controller = using_slip_controller
        self.using_slip_detection = using_slip_detection
  
        rospy.loginfo("initializing "+whicharm+" controller manager")
        rospy.loginfo("controller manager: using_slip_controller:"+str(using_slip_controller))
        rospy.loginfo("controller manager: using_slip_detection:"+str(using_slip_detection))
  
        #wait for the load/unload/switch/list controller services to be there and initialize their service proxies
        rospy.loginfo("controller manager waiting for pr2_controller_manager services to be there")
        load_controller_serv_name = 'pr2_controller_manager/load_controller'
        unload_controller_serv_name = 'pr2_controller_manager/unload_controller'
        switch_controller_serv_name = 'pr2_controller_manager/switch_controller'
        list_controllers_serv_name = 'pr2_controller_manager/list_controllers'
        self.wait_for_service(load_controller_serv_name)
        self.wait_for_service(unload_controller_serv_name)
        self.wait_for_service(switch_controller_serv_name)
        self.wait_for_service(list_controllers_serv_name)
        self.load_controller_service = \
            rospy.ServiceProxy(load_controller_serv_name, LoadController)
        self.unload_controller_service = \
            rospy.ServiceProxy(unload_controller_serv_name, UnloadController)
        self.switch_controller_service = \
            rospy.ServiceProxy(switch_controller_serv_name, SwitchController)
        self.list_controllers_service = \
            rospy.ServiceProxy(list_controllers_serv_name, ListControllers)
  
        #initialize listener for JointStates messages (separate thread)
        self.joint_states_listener = joint_states_listener.LatestJointStates()
  
        #joint trajectory action client
        joint_trajectory_action_name = whicharm+'_arm_controller/joint_trajectory_action'
        self.joint_action_client = \
            actionlib.SimpleActionClient(joint_trajectory_action_name, JointTrajectoryAction)
  
        #slip-sensing gripper action client
        if self.using_slip_controller:
          gripper_action_name = whicharm+'_gripper_sensor_controller/gripper_action'
          self.gripper_action_client = actionlib.SimpleActionClient(gripper_action_name, Pr2GripperCommandAction)
  
        #default gripper action client
        else:
          gripper_action_name = whicharm+'_gripper_controller/gripper_action'
          self.gripper_action_client = \
              actionlib.SimpleActionClient(gripper_action_name, Pr2GripperCommandAction)
  
        #other slip-sensing gripper actions
        if self.using_slip_detection:
          gripper_find_contact_action_name = whicharm+'_gripper_sensor_controller/find_contact'
          gripper_grab_action_name = whicharm+'_gripper_sensor_controller/grab'
          gripper_event_detector_action_name = whicharm+'_gripper_sensor_controller/event_detector'
  
          self.gripper_find_contact_action_client = actionlib.SimpleActionClient(gripper_find_contact_action_name, \
                                                                                   PR2GripperFindContactAction)
          self.gripper_grab_action_client = actionlib.SimpleActionClient(gripper_grab_action_name, \
                                                                                 PR2GripperGrabAction)
          self.gripper_event_detector_action_client = actionlib.SimpleActionClient(gripper_event_detector_action_name, \
                                                                                 PR2GripperEventDetectorAction)
  
        #move arm client is filled in the first time it's called
        self.move_arm_client = None
        
        #wait for the joint trajectory and gripper action servers to be there
        self.wait_for_action_server(self.joint_action_client, joint_trajectory_action_name)
        self.wait_for_action_server(self.gripper_action_client, gripper_action_name)
        if self.using_slip_detection:
          self.wait_for_action_server(self.gripper_find_contact_action_client, gripper_find_contact_action_name)
          self.wait_for_action_server(self.gripper_grab_action_client, gripper_grab_action_name)
          self.wait_for_action_server(self.gripper_event_detector_action_client, gripper_event_detector_action_name)
  
        #initialize a tf listener
        if tf_listener == None:
          self.tf_listener = tf.TransformListener()
        else:
          self.tf_listener = tf_listener

        #which Cartesian controller to use
        self.use_trajectory_cartesian = rospy.get_param('~use_trajectory_cartesian', 1)
        self.use_task_cartesian = rospy.get_param('~use_task_cartesian', 0)
        if self.use_trajectory_cartesian and self.use_task_cartesian:
          rospy.loginfo("can't use both trajectory and task controllers!  Defaulting to task")
          self.use_trajectory_cartesian = 0
        rospy.loginfo("use_trajectory_cartesian: "+str(self.use_trajectory_cartesian))
        rospy.loginfo("use_task_cartesian: "+str(self.use_task_cartesian))
  
        #names of the controllers
        if self.use_trajectory_cartesian:
            self.cartesian_controllers = ['_arm_cartesian_pose_controller', 
                                            '_arm_cartesian_trajectory_controller']
            self.cartesian_controllers = [self.whicharm+x for x in self.cartesian_controllers]
        else:
            self.cartesian_controllers = [self.whicharm + '_cart']
        self.joint_controller = self.whicharm+'_arm_controller'
        if self.using_slip_controller:
          self.gripper_controller = self.whicharm+'_gripper_sensor_controller'
        else:
          self.gripper_controller = self.whicharm+'_gripper_controller'
  
        #parameters for the Cartesian controllers    
        if self.use_trajectory_cartesian:
            self.cart_params = JTCartesianParams(self.whicharm)
        else:
            self.cart_params = JTCartesianTaskParams(self.whicharm, self.use_task_cartesian)
  
        #parameters for the joint controller
        self.joint_params = JointParams(self.whicharm)
  
        #load the Cartesian controllers if not already loaded
        rospy.loginfo("loading any unloaded Cartesian controllers")
        self.load_cartesian_controllers()
        time.sleep(2)
        rospy.loginfo("done loading controllers")
  
        #services for the J-transpose Cartesian controller 
        if self.use_trajectory_cartesian:
            cartesian_check_moving_name = whicharm+'_arm_cartesian_trajectory_controller/check_moving'
            cartesian_move_to_name = whicharm+'_arm_cartesian_trajectory_controller/move_to'
            cartesian_preempt_name = whicharm+'_arm_cartesian_trajectory_controller/preempt'
            self.wait_for_service(cartesian_check_moving_name)      
            self.wait_for_service(cartesian_move_to_name)
            self.wait_for_service(cartesian_preempt_name)      
            self.cartesian_moving_service = rospy.ServiceProxy(cartesian_check_moving_name, CheckMoving)
            self.cartesian_cmd_service = rospy.ServiceProxy(cartesian_move_to_name, MoveToPose)
            self.cartesian_preempt_service = rospy.ServiceProxy(cartesian_preempt_name, Empty)
        else:
            cartesian_cmd_name = whicharm+'_cart/command_pose'
            self.cartesian_cmd_pub = rospy.Publisher(cartesian_cmd_name, PoseStamped)
  
        #re-load the Cartesian controllers with the gentle params
        self.cart_params.set_params_to_gentle()
        self.reload_cartesian_controllers()
  
        #create an IKUtilities class object
        rospy.loginfo("creating IKUtilities class objects")
        rospy.set_param("~collision_aware_ik", 0) # TODO CHANGE
        if whicharm == 'r':
          self.ik_utilities = HRLIKUtilities('right', self.tf_listener) # TODO CHANGE
        else:
          self.ik_utilities = HRLIKUtilities('left', self.tf_listener) # TODO CHANGE
        rospy.loginfo("done creating IKUtilities class objects")
  
        #joint names for the arm
        joint_names = ["_shoulder_pan_joint", 
                       "_shoulder_lift_joint", 
                       "_upper_arm_roll_joint", 
                       "_elbow_flex_joint", 
                       "_forearm_roll_joint", 
                       "_wrist_flex_joint", 
                       "_wrist_roll_joint"]
        self.joint_names = [whicharm + x for x in joint_names]
  
        #thread to send clipped versions of the last Cartesian command
        if not self.use_trajectory_cartesian:
          self.cartesian_command_lock = threading.Lock()
          self.cartesian_command_thread_running = False
          self.cartesian_desired_pose = self.get_current_wrist_pose_stamped('/base_link')
          self.cartesian_command_thread = threading.Thread(target=self.cartesian_command_thread_func)
          self.cartesian_command_thread.setDaemon(True)
          self.cartesian_command_thread.start()
        rospy.loginfo("done with controller_manager init for the %s arm"%whicharm)

    def command_interpolated_ik(self, end_pose, start_pose = None, collision_aware = 1, step_size = .02, max_joint_vel = .05, joints_bias = None, bias_radius = 0.0):
        self.check_controllers_ok('joint')

        #find the current joint angles 
        current_angles = self.get_current_arm_angles()

        if start_pose == None:
            #use the current wrist pose as the start pose/angles
            (current_trans, current_rot) = self.return_cartesian_pose()

            #put the current pose into a poseStamped
            start_pose = create_pose_stamped(current_trans+current_rot)
            #print "start_pose:\n", start_pose
        #print "end_pose:\n", end_pose

        #find a collision-free joint trajectory to move from the current pose 
        #to the desired pose using Cartesian interpolation
        (trajectory, error_codes) = self.ik_utilities.check_cartesian_path(start_pose, \
                                         end_pose, current_angles, step_size, .1, math.pi/4, collision_aware, \
                                         joints_bias = joints_bias, bias_radius = bias_radius,
                                         steps_before_abort=-1)

        #if some point is not possible, quit
        if any(error_codes):
            rospy.loginfo("can't execute an interpolated IK trajectory to that pose")
            return 0
        trajectory = np.array(trajectory)[np.where(np.array(error_codes) == 0)].tolist()

#         print "found trajectory:"
#         for point in trajectory:
#             if type(point) == tuple:
#                 print self.pplist(point)
#             else:
#                 print point

        #send the trajectory to the joint trajectory action
        #print "moving through trajectory"
        self.command_joint_trajectory(trajectory, max_joint_vel)
        return 1

    ##move in Cartesian space using IK Cartesian interpolation
    #if collision_aware, quits if the path is not collision-free
    #if blocking, waits for completion, otherwise returns after sending the goal
    #step_size is the step size for interpolation
    #pos_thres and rot_thres are thresholds within which the goal is declared reached
    #times out and quits after timeout, and waits settling_time after the 
    #controllers declare that they're done
    def move_cartesian_ik(self, goal_pose, collision_aware = 0, blocking = 1,
                       step_size = .005, pos_thres = .02, rot_thres = .1,
                       timeout = rospy.Duration(30.), 
                       settling_time = rospy.Duration(0.25), vel = .15,
                       joints_bias = None, bias_radius = 0.0):
  
        #send the interpolated IK goal to the joint trajectory action
        result = self.command_interpolated_ik(goal_pose, collision_aware = collision_aware, \
                                                step_size = step_size, max_joint_vel = vel, \
                                                joints_bias = joints_bias, \
                                                bias_radius = bias_radius)
        if not result:
            return "no solution"
        if not blocking:
            return "sent goal"
  
        #blocking: wait for the joint trajectory action to get there or time out
        joint_action_result = self.wait_joint_trajectory_done(timeout)
        if joint_action_result == "timed out":
            return "timed out"
        
        if self.check_cartesian_really_done(goal_pose, pos_thres, rot_thres):
            return "success"
        return "failed"

    def move_arm_pose_biased(self, pose_stamped, joints_bias = [0.]*7, max_joint_vel=0.2,
                                        blocking = 1, init_angs=[0.]*7):
        init_pos = [pose_stamped.pose.position.x, pose_stamped.pose.position.y, 
                    pose_stamped.pose.position.z] 

        for i in range(10):
            cur_pos = np.array(init_pos) + np.random.uniform(-i * 0.0015, i * 0.0015, 3)
            pose_stamped.pose.position.x = cur_pos[0]
            pose_stamped.pose.position.y = cur_pos[1]
            pose_stamped.pose.position.z = cur_pos[2]
            solution = self.ik_utilities.run_biased_ik(pose_stamped, joints_bias, 
                                                       init_angs=init_angs)
            if solution:
                break

        if not solution:
            rospy.logerr("no IK solution found for goal pose!")
            return None
        else:
            result = self.command_joint_trajectory([solution], blocking=blocking, 
                                                   max_joint_vel=max_joint_vel)
            return True
