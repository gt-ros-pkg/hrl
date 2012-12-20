#!/usr/bin/env python

import math, numpy as np
import sys, optparse

import interactive_marker_util as imu

import roslib; roslib.load_manifest('hrl_tactile_controller')
import rospy

import hrl_lib.transforms as tr

import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs
import std_msgs.msg

import interactive_markers.interactive_marker_server as ims
import interactive_markers.menu_handler as mh

from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerFeedback, InteractiveMarkerControl
from geometry_msgs.msg import Pose, PoseStamped, PointStamped, PoseArray
from std_msgs.msg import String, Bool, Empty
from hrl_haptic_manipulation_in_clutter_msgs.msg import ObjectInfo
import RRT
import TaskSpaceFuncs as TSFs
import initConfig as ic

# stop current controller and then allow a new controller to execute.

class MPCTeleopInteractiveMarkers():
  def __init__(self, opt):
    self.opt = opt

  def stop_start_epc(self):
    # stop current controller
    self.stop_pub.publish(Bool(True))
    rospy.sleep(0.3)
    # allow controller to start.
    self.stop_pub.publish(Bool(False))
    rospy.sleep(0.1)
  
  def wp_feedback_rviz_cb(self, feedback):
#    print "waypoint moved"
    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
      ps = PoseStamped()
      ps.header.frame_id = feedback.header.frame_id

      pp = feedback.pose.position
      qq = feedback.pose.orientation

      quat = [qq.x, qq.y, qq.z, qq.w]
      r = tr.quaternion_to_matrix(quat)
      offset = np.matrix([0.0, 0., 0.]).T
      o_torso = r * offset

      ps.pose.position.x = pp.x + o_torso[0,0]
      ps.pose.position.y = pp.y + o_torso[1,0]
      ps.pose.position.z = pp.z + o_torso[2,0]

      ps.pose.orientation.x = qq.x
      ps.pose.orientation.y = qq.y
      ps.pose.orientation.z = qq.z
      ps.pose.orientation.w = qq.w
      
      self.current_goal_pose = ps
      self.wp_pose_pub.publish(ps)
    
    self.server.applyChanges()
  
  def wp_feedback_go_handler(self, feedback):
    self.stop_start_epc()
    rospy.loginfo("MPC Teleop: Publishing new goal. Position only.")
#    print "feedback"
    weights_msg = haptic_msgs.HapticMpcWeights()
    weights_msg.header.stamp = rospy.Time.now()
    weights_msg.pos_weight = 5.0
    weights_msg.orient_weight = 0.0
    self.mpc_weights_pub.publish(weights_msg) # Enable position tracking only - disable orientation by setting the weight to 0 
    print "Current Goal:", self.current_goal_pose
    self.goal_pos_pub.publish(self.current_goal_pose)
    self.ros_pub.publish('go_to_way_point')

  def poseCheckCallback(self, data):
    xdiff = self.initPos[0] - data.pose.position.x
    ydiff = self.initPos[1] - data.pose.position.y
    zdiff = self.initPos[2] - data.pose.position.z
    totaldiff = np.sqrt(xdiff**2 + ydiff **2 + zdiff**2)
    #print self.REACHED_TO_INIT
    if totaldiff < 0.2:  
      self.REACHED_TO_INIT = True
      self.index = self.index + 1
    else:
      self.REACHED_TO_INIT = False
      self.index = 0
      
    #if (self.REACHED_TO_INIT == True) and (self.index == 1):
        #self.stop_start_epc()
        #ps = Pose()
        #self.psList = PoseArray()
        #self.psList.header.frame_id = "/torso_lift_link"
        #self.psList.header.stamp = rospy.Time.now()
        #self.paths = RRT.RRT(self.initPos, self.goalPos, self.obsDict, self.limits, TSFs.dist, TSFs.validation, TSFs.extension)
        #print self.initPos
        #print self.goalPos
        #j = 0 
        #for path in self.paths:
            #print path
            #ps.position.x = path[0]
            #ps.position.y = path[1]
            #ps.position.z = path[2]
            #ps.orientation.x = 0.
            #ps.orientation.y = 0.
            #ps.orientation.z = 0.
            #ps.orientation.w = 1.
            #self.psList.poses.append(ps)
            #ps = Pose()
            #j = j+1

        #print "RRT Step-Size:", j
        #print "RRT Steps", self.psList

        #rospy.loginfo("Running RRT")

        #self.goal_traj_pub.publish(self.psList)
        #rospy.sleep(0.1)
    
  def rrtcallback(self, data): 
    if ((data.obj_id == 1) and (self.GOING_BACK == True)):
      self.stop_start_epc()
      self.goal_traj_pub.publish(PoseArray())
      rospy.sleep(0.1)
      rospy.loginfo("Stopping MPC due to the detection of a Rigid and Fixed Obstacle")
      radius = 0.15
      if (abs(self.pos_x - data.contact_x) > 0.01) and (abs(self.pos_y - data.contact_y) > 0.01) and (abs(self.pos_z - data.contact_z) > 0.01):
        self.obsDict[self.key] = [radius, data.contact_x, data.contact_y, data.contact_z]
        self.pos_x = data.contact_x
        self.pos_y = data.contact_y
        self.pos_z = data.contact_z

        print '###############'
        print self.obsDict[self.key]
        print '###############'
      
        self.key = self.key + 1
        #self.index = 0

      rospy.loginfo("Going back to Initial Position")
      self.stop_start_epc()
      self.goal_pos_pub.publish(self.psInit)
      self.GOING_BACK = False
      #rospy.sleep(0.1)

    self.goal_pos_pub.publish(self.current_goal_pose)
    self.ros_pub.publish('go_to_way_point')

############ RRT part begin ##########################################

  def wp_feedback_go_rrt_handler(self, feedback):
    self.stop_start_epc()
    rospy.loginfo("MPC Teleop with RRT: Publishing new goal. Position only.")
#    print "feedback"
    weights_msg = haptic_msgs.HapticMpcWeights()
    weights_msg.header.stamp = rospy.Time.now()
    weights_msg.pos_weight = 5.0
    weights_msg.orient_weight = 0.0
    self.mpc_weights_pub.publish(weights_msg) # Enable position tracking only - disable orientation by setting the weight to 0
    
    ps = Pose()
    self.psGoal = PoseStamped()
    self.psInit = PoseStamped()
    self.psInit.header = self.wp_im.header
    self.psList = PoseArray()

    self.psGoal = self.current_goal_pose
    self.goalPos[0] = self.psGoal.pose.position.x
    self.goalPos[1] = self.psGoal.pose.position.y
    self.goalPos[2] = self.psGoal.pose.position.z
    
    self.goalPos[0] = 0.92
    self.goalPos[1] = 0.23
    self.goalPos[2] = 0.01

    #print self.psGoal.pose.orientation.x, self.psGoal.pose.orientation.y, self.psGoal.pose.orientation.z
    #print self.psGoal.pose.position.x, self.psGoal.pose.position.y, self.psGoal.pose.position.z
    print "Goal position:"
    print self.goalPos

    # Get Initial Position from Daehyung's Work
    self.initPos, initOrt = ic.getInitPos(self.goalPos)
    self.initPos[0] = 0.345234096022
    self.initPos[1] = -0.044929906316
    self.initPos[2] = -0.0692195939604
                        
    # Add a function to go to the initPos
    self.psInit.pose.position.x = self.initPos[0]
    self.psInit.pose.position.y = self.initPos[1]
    self.psInit.pose.position.z = self.initPos[2]

    print "Initial position based on given Goal:"
    print self.psInit.pose.position.x, self.psInit.pose.position.y, self.psInit.pose.position.z

    self.psInit.pose.orientation.x = 0
    self.psInit.pose.orientation.y = 0
    self.psInit.pose.orientation.z = 0
    self.psInit.pose.orientation.w = 1
    
    if self.REACHED_TO_INIT == False:  
        self.goal_pos_pub.publish(self.psInit)
        self.ros_pub.publish('go_to_way_point')
        rospy.sleep(5)

    # Implement RRT
    if self.REACHED_TO_INIT == True:
        self.psList.header.frame_id = "/torso_lift_link"
        self.psList.header.stamp = rospy.Time.now()

        print "Set of Obstacles:", self.obsDict

        #self.goal_pos_pub.publish(self.psInit)
        #self.ros_pub.publish('go_to_way_point')
        #rospy.sleep(0.1)

        self.paths = RRT.RRT(self.initPos, self.goalPos, self.obsDict, self.limits, TSFs.dist, TSFs.validation, TSFs.extension)
        j = 0 
        for path in self.paths:
            print path
            ps.position.x = path[0]
            ps.position.y = path[1]
            ps.position.z = path[2]
            ps.orientation.x = 0.
            ps.orientation.y = 0.
            ps.orientation.z = 0.
            ps.orientation.w = 1.
            self.psList.poses.append(ps)
            ps = Pose()
            j = j+1

        print "RRT Step-Size:", j
        print "RRT Steps", self.psList

        rospy.loginfo("Running RRT")
        self.GOING_BACK = True
        self.goal_traj_pub.publish(self.psList)
        #rospy.sleep(0.1)

    ps = PoseStamped()
    psList = PoseArray()

    ps.header.frame_id = feedback.header.frame_id
    psList.header.frame_id = feedback.header.frame_id

    pp = feedback.pose.position
    qq = feedback.pose.orientation

    initPos = [0.0, 0.0, 0.0] 
    goalPos = [9.0, 0.0, 0.0] 
    obsDict = {1:[1, 3.0, 0.0, 0.0], 2:[5, 0.0, 7.0, 0.0]} 
    limits = [[-10.0, -10.0, -10.0], [10.0, 10.0, 10.0]]

    paths = RRT.RRT(initPos, goalPos, obsDict, limits)
    j = 0 
    for path in paths:
        ps.pose.position.x = path[0]
        ps.pose.position.y = path[0]
        ps.pose.position.z = path[0]
        ps.pose.orientation.x = 0.
        ps.pose.orientation.y = 0.
        ps.pose.orientation.z = 0.
        ps.pose.orientation.w = 1.
        j = j+1
        psList.append(ps)

    #self.goal_traj_pub.publish(psList)
    print "RRT Steps:", j
    rospy.loginfo("Running RRT")

############ RRT part end ##########################################
      
  def wp_feedback_reduce_forces(self,feedback):
    self.stop_start_epc()
    self.ros_pub.publish('reduce_force')
  
  def wp_feedback_start_darpa_trial(self, feedback):
    self.ros_pub.publish('go_darpa_goals')
  
  def wp_feedback_orient_handler(self, feedback):
    self.stop_start_epc()
    rospy.loginfo("MPC Teleop: Publishing new goal. Position and Orientation.")
#    print "feedback"
    weights_msg = haptic_msgs.HapticMpcWeights()
    weights_msg.header.stamp = rospy.Time.now()
    weights_msg.pos_weight = 5.0
    weights_msg.orient_weight = 4.0
    self.mpc_weights_pub.publish(weights_msg) # Enable position and orientation tracking 
    self.goal_pos_pub.publish(self.current_goal_pose)  
    self.ros_pub.publish('orient_to_way_point')
    self.add_topic_pub = rospy.Publisher('/haptic_mpc/add_taxel_array', std_msgs.msg.String)
    self.remove_topic_pub = rospy.Publisher('/haptic_mpc/remove_taxel_array', std_msgs.msg.String)
  
  def wp_feedback_stop_handler(self, feedback):
    self.stop_start_epc()
    self.goal_traj_pub.publish(PoseArray())
    rospy.loginfo("Stopping MPC")
    

  def enablePps(self):
    self.add_topic_pub.publish('/pr2_pps_right_sensor/taxels/forces')
    self.add_topic_pub.publish('/pr2_pps_left_sensor/taxels/forces')

  def disablePps(self):
    self.remove_topic_pub.publish('/pr2_pps_right_sensor/taxels/forces')
    self.remove_topic_pub.publish('/pr2_pps_left_sensor/taxels/forces')

  def wp_feedback_open_handler(self, feedback):
    self.open_pub.publish(Empty())
    self.enablePps()

  def wp_feedback_close_handler(self, feedback):
    self.disablePps()
    self.close_pub.publish(Empty())
  
  def wp_feedback_disable_handler(self, feedback):
    self.disable_pub.publish(Empty())
  
  def wp_feedback_enable_handler(self, feedback):
    self.enable_pub.publish(Empty())

  def wp_feedback_zero_handler(self, feedback):
    self.zero_gripper_pub.publish(Empty())
    self.zero_gripper_right_link_pub.publish(Empty())
    self.zero_gripper_left_link_pub.publish(Empty())
    self.zero_gripper_palm_pub.publish(Empty())
    self.zero_forearm_pub.publish(Empty())
    self.zero_upperarm_pub.publish(Empty())
    self.zero_pps_left_pub.publish(Empty())
    self.zero_pps_right_pub.publish(Empty())
  
  def goal_feedback_rviz_cb(self, feedback):
#    print "goal_feedback_rviz"
    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
      ps = PoseStamped()
      ps.header.frame_id = feedback.header.frame_id

      pp = feedback.pose.position
      ps.pose.position.x = pp.x
      ps.pose.position.y = pp.y
      ps.pose.position.z = pp.z

      qq = feedback.pose.orientation
      ps.pose.orientation.x = qq.x
      ps.pose.orientation.y = qq.y
      ps.pose.orientation.z = qq.z
      ps.pose.orientation.w = qq.w

      #goal_pos_pub.publish(ps)
    
    self.server.applyChanges()
  
  def goal_feedback_menu_handler(self, feedback):
    if feedback.menu_entry_id == 1:
      self.ros_pub.publish('set_goal')
    elif feedback.menu_entry_id == 2:
      self.stop_start_epc()
      self.ros_pub.publish('reach_to_goal')
    elif feedback.menu_entry_id == 3:
      self.pause_pub.publish(Bool(True))
    elif feedback.menu_entry_id == 4:
      self.pause_pub.publish(Bool(False))
  
  def initComms(self, node_name):
    rospy.init_node(node_name)
    
    # Goal pose publisher.
    self.goal_pos_pub = rospy.Publisher("/haptic_mpc/goal_pose", PoseStamped, latch=True)
    self.mpc_weights_pub = rospy.Publisher("/haptic_mpc/weights", haptic_msgs.HapticMpcWeights)
    self.goal_traj_pub = rospy.Publisher("/haptic_mpc/goal_pose_array", PoseArray)

    self.add_topic_pub = rospy.Publisher("/haptic_mpc/add_taxel_array", std_msgs.msg.String)
    self.remove_topic_pub = rospy.Publisher("/haptic_mpc/remove_taxel_array", std_msgs.msg.String)


    # These are deprecated and should be cleaned up.
    self.wp_pose_pub = rospy.Publisher('/teleop_rviz/command/way_point_pose', PoseStamped)
    self.ros_pub = rospy.Publisher('/epc_skin/command/behavior', String)
    self.pause_pub = rospy.Publisher('/epc/pause', Bool)
    self.stop_pub = rospy.Publisher('/epc/stop', Bool)
    
    self.open_pub = rospy.Publisher('open_gripper', Empty)
    self.close_pub = rospy.Publisher('close_gripper', Empty)
    
    self.disable_pub = rospy.Publisher('/pr2_fabric_gripper_sensor/disable_sensor', Empty)
    self.enable_pub = rospy.Publisher('/pr2_fabric_gripper_sensor/enable_sensor', Empty)
    
    self.zero_gripper_pub = rospy.Publisher('/pr2_fabric_gripper_sensor/zero_sensor', Empty)
    self.zero_gripper_left_link_pub = rospy.Publisher('/pr2_fabric_gripper_left_link_sensor/zero_sensor', Empty)
    self.zero_gripper_right_link_pub = rospy.Publisher('/pr2_fabric_gripper_right_link_sensor/zero_sensor', Empty)
    self.zero_gripper_palm_pub = rospy.Publisher('/pr2_fabric_gripper_palm_sensor/zero_sensor', Empty)
    self.zero_forearm_pub = rospy.Publisher('/pr2_fabric_forearm_sensor/zero_sensor', Empty)
    self.zero_upperarm_pub = rospy.Publisher('/pr2_fabric_upperarm_sensor/zero_sensor', Empty)
    self.zero_pps_left_pub = rospy.Publisher('/pr2_pps_left_sensor/zero_sensor', Empty)
    self.zero_pps_right_pub = rospy.Publisher('/pr2_pps_right_sensor/zero_sensor', Empty)
    
    self.server = ims.InteractiveMarkerServer('teleop_rviz_server')

  def initMarkers(self):
    pos = np.matrix([0.,0.,0.]).T
    ps = PointStamped()
    ps.header.frame_id = '/torso_lift_link'

    #--- interactive marker for way point ---
    if self.opt.cody:
      ps.point.x = 0.4
      ps.point.y = -0.1
      ps.point.z = -0.15
      if opt.orientation:
        self.wp_im = imu.make_6dof_gripper(False, ps, 0.28, (1., 1., 0.,0.4), "cody")
        #wp_im = imu.make_6dof_marker(False, ps, 0.15, (1., 1., 0.,0.4), 'sphere')
      else:
        self.wp_im = imu.make_3dof_marker_position(ps, 0.15, (1., 1., 0.,0.4), 'sphere')
    elif opt.pr2:
      ps.point.x = 0.6
      ps.point.y = -0.1
      ps.point.z = -0.15
      if opt.orientation:
        #wp_im = imu.make_6dof_marker(False, ps, 0.15, (1., 1., 0.,0.4), 'sphere')
        self.wp_im = imu.make_6dof_gripper(False, ps, 0.28, (1., 1., 0.,0.4))
      else:
        self.wp_im = imu.make_3dof_marker_position(ps, 0.15, (1., 1., 0.,0.4), 'sphere')
    elif opt.sim:
      ps.point.x = 0.4
      ps.point.y = -0.1
      ps.point.z = 0.15
      self.wp_im = imu.make_marker_position_xy(ps, 0.15, (1., 1., 0.,0.4), 'sphere')
    elif self.opt.simcody:
      ps.point.x = 0.4
      ps.point.y = -0.1
      ps.point.z = -0.15
      if opt.orientation:
        self.wp_im = imu.make_6dof_gripper(False, ps, 0.28, (1., 1., 0.,0.4), "cody")
        #wp_im = imu.make_6dof_marker(False, ps, 0.15, (1., 1., 0.,0.4), 'sphere')
      else:
        self.wp_im = imu.make_3dof_marker_position(ps, 0.15, (1., 1., 0.,0.4), 'sphere')
    else:
      rospy.logerr('Please specify a testbed: --cody, --pr2, --sim, or --simcody')
      sys.exit()
  
    ps = PoseStamped()
    ps.header = self.wp_im.header
    ps.pose = self.wp_im.pose
    self.current_goal_pose = ps
     
    self.wp_im.name = 'way_point'
    self.wp_im.description = 'Way Point'
    self.server.insert(self.wp_im, self.wp_feedback_rviz_cb)
    self.server.applyChanges()
    
  def initMenu(self):
    self.wp_menu_handler = mh.MenuHandler()
    self.wp_menu_handler.insert('Go', callback = self.wp_feedback_go_handler)
    self.wp_menu_handler.insert('Go RRT', callback = self.wp_feedback_go_rrt_handler)
    self.wp_menu_handler.insert('Orient', callback = self.wp_feedback_orient_handler)
    self.wp_menu_handler.insert('Stop', callback = self.wp_feedback_stop_handler)
    self.wp_menu_handler.insert('Open Gripper', callback = self.wp_feedback_open_handler)
    self.wp_menu_handler.insert('Close Gripper', callback = self.wp_feedback_close_handler)
    self.wp_menu_handler.insert('Zero Skin', callback = self.wp_feedback_zero_handler)
    #self.wp_menu_handler.insert('Reduce Forces', callback = self.wp_feedback_reduce_forces)
    
    imu.add_menu_handler(self.wp_im, self.wp_menu_handler, self.server)
  
  #    #--- interactive marker for goal ---
  #    if opt.cody:
  #        ps.point.x = 0.5
  #        ps.point.y = -0.2
  #        ps.point.z = -0.15
  #        if opt.orientation:
  #            goal_im = imu.make_6dof_marker(False, ps, 0.15, (0., 1., 1.,0.4), 'sphere')
  #        else:
  #            goal_im = imu.make_3dof_marker_position(ps, 0.15, (0., 1., 1.,0.4), 'sphere')
  #    elif opt.pr2:
  #        ps.point.x = 0.7
  #        ps.point.y = -0.2
  #        ps.point.z = -0.15
  #        if opt.orientation:
  #            goal_im = imu.make_6dof_marker(False, ps, 0.15, (0., 1., 1.,0.4), 'sphere')
  #        else:
  #            goal_im = imu.make_3dof_marker_position(ps, 0.15, (0., 1., 1.,0.4), 'sphere')
  #    elif opt.sim:
  #        ps.point.x = 0.5
  #        ps.point.y = -0.2
  #        ps.point.z = 0.15
  #        goal_im = imu.make_marker_position_xy(ps, 0.15, (0., 1., 1.,0.4), 'sphere')
  #
  #    goal_im.name = 'goal'
  #    goal_im.description = 'Goal'
  #    server.insert(goal_im, goal_feedback_rviz_cb)
  #    server.applyChanges()
  #
  #    goal_menu_handler = mh.MenuHandler()
  #    goal_menu_handler.insert('Set', callback = goal_feedback_menu_handler)
  #    goal_menu_handler.insert('Go', callback = goal_feedback_menu_handler)
  #    goal_menu_handler.insert('Pause', callback = goal_feedback_menu_handler)
  #    goal_menu_handler.insert('Resume', callback = goal_feedback_menu_handler)
  #    imu.add_menu_handler(goal_im, goal_menu_handler, server)    
  
  def start(self):
<<<<<<< HEAD
<<<<<<< HEAD

    self.key = 1
    self.obsDict = {}
    self.initPos = [0.5, 0.0, 0.0]
    self.goalPos = [1.2, 0.0, 0.0]
    self.limits = [[0.15, -0.1, -0.1], [1.2, 1.2, 0.1]] 
    self.pos_x = 0
    self.pos_y = 0
    self.pos_z = 0
    self.REACHED_TO_INIT = False
    self.GOING_BACK = False
    self.index = 0
    rospy.loginfo('Haptic MPC interactive marker server started : Getting data from HMM based online classification')
    rospy.Subscriber("/hmm/object_data", ObjectInfo, self.rrtcallback)
    rospy.Subscriber("/haptic_mpc/gripper_pose", PoseStamped, self.poseCheckCallback)
    rospy.spin()

if __name__ == '__main__':
  p = optparse.OptionParser()

  p.add_option('--cody', action='store_true', dest='cody',
               help='display interactive markers for cody')
  p.add_option('--pr2', action='store_true', dest='pr2',
               help='display interactive markers for the PR2')
  p.add_option('--sim', action='store_true', dest='sim',
               help='disaply interactive markers for software simulation')
  p.add_option('--simcody', action='store_true', dest='simcody',
               help='disaply interactive markers for simcody simulation')
  p.add_option('--orientation', action='store_true', dest='orientation',
               help='command orientation as well')

  opt, args = p.parse_args()

  # Initialise publishers/subscribers
  mpc_ims = MPCTeleopInteractiveMarkers(opt)
  mpc_ims.initComms("mpc_teleop_rviz")
  mpc_ims.initMarkers()
  mpc_ims.initMenu()
  mpc_ims.start()
  



