## @package hrl_haptic_mpc
# 
# @author Jeff Hawke jhawke@gatech.edu
# @version 0.1
# @copyright Simplified BSD Licence

import roslib
roslib.load_manifest("hrl_tactile_controller")
import rospy
import tf

import hrl_lib.transforms as tr
import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs
import geometry_msgs.msg
import std_msgs.msg

import numpy as np
import threading, copy
import sys

## Provides an interface to process TaxelArrays published on multiple topics, including trimming the data and transforming the reference frame
class TaxelArrayClient(): 
  ## Constructor
  # @param skin_topic_list List of strings specifying the topics to subscribe to for TaxelArray messages
  # @param torso_frame String indicating which frame ID is the base frame for the arm
  # @param tf_listener TF Listener object, if one exists. If this is None, the class will create one.
  def __init__(self, skin_topic_list, torso_frame="/torso_lift_link", tf_listener=None):
    ## Lock for skin data structure.
    self.data_lock = threading.RLock() 
    ## Lock for skin data topics.
    self.topic_lock = threading.RLock() 
    
    ## torso_frame Torso frame ID used as the base frame for the associated arm, eg, "/torso_lift_link"
    self.torso_frame = torso_frame 
    ## Threshold used to control how the data is trimmed - should be set by whatever instatiates a TaxelArrayClient. Default: 0.0 (ie, trim nothing)
    self.trim_threshold = 0.0    

    ## Dictionary containing the skin subscribers, indexed by topic name
    self.skin_subs = {} 
    ## Dictionary containing the raw TaxelArray messages heard by the client, indexed by topic name
    self.skin_data = {} 
    ## Dictionary containing the processed TaxelArray messages heard by the client, indexed by topic name
    self.trimmed_skin_data = {} 

    try:
      if tf_listener == None:
        ## TF Listener
        self.tf_lstnr = tf.TransformListener()
      else:
        ## TF Listener
        self.tf_lstnr = tf_listener
    except rospy.ServiceException, e:
      rospy.loginfo("ServiceException caught while instantiating a TF listener. Seems to be normal")
      pass  

    ## List of skin topics used by the client
    self.skin_topic_list = skin_topic_list
    # Initialise a subscriber for each topic in the list
    for skin_topic in self.skin_topic_list:
      self.addSkinTopic(skin_topic)

    ## Add skin topic subscriber.
    rospy.Subscriber("/haptic_mpc/add_taxel_array", std_msgs.msg.String, self.addSkinTopicCallback)
    ## Remove skin topic subscriber
    rospy.Subscriber("/haptic_mpc/remove_taxel_array", std_msgs.msg.String, self.removeSkinTopicCallback)

    ## Current topics ROS publisher. Publishes the list of topics on change.
    self.current_topics_pub = rospy.Publisher("/haptic_mpc/skin_topics", haptic_msgs.StringArray, latch=True)
    self.current_topics_pub.publish(self.skin_data.keys())
  
  ## Set the trim threshold used by the client. Should be greater or equal to 0.0.
  # @param threshold Desired threshold. Should be greater than or equal to 0.0.
  def setTrimThreshold(self, threshold):
    self.trim_threshold = threshold

  ## Callback function which sets the topic. 
  # @param msg std_msgs/String message. 
  def addSkinTopicCallback(self, msg):
    rospy.loginfo("Adding skin TaxelArray topic: %s" % str(msg.data))
    self.addSkinTopic(msg.data)
    rospy.loginfo("Current skin topics: \n%s", str(self.skin_subs.keys()))
    self.current_topics_pub.publish(self.skin_subs.keys())

  ## Callback function to removed skin topic.
  # @param msg StringArray message. 
  def removeSkinTopicCallback(self, msg):
    rospy.loginfo("Removing skin TaxelArray topic: %s" % str(msg.data))
    self.removeSkinTopic(msg.data)
    rospy.loginfo("Current skin topics: \n%s", str(self.skin_subs.keys()))
    self.current_topics_pub.publish(self.skin_subs.keys())

  ## Add skin topic to internal data structures.
  # @param skin_topic String specifying the topic to be added.
  def addSkinTopic(self, skin_topic):
    if skin_topic in self.skin_subs.keys():
        return
    with self.topic_lock:
      self.skin_topic_list.append(skin_topic)
      self.skin_data[skin_topic] = haptic_msgs.TaxelArray()
      self.skin_subs[skin_topic] = rospy.Subscriber(skin_topic, haptic_msgs.TaxelArray, self.skinCallback, skin_topic)

  ## Remove skin topic from internal data structures.
  # @param skin_topic String specifying the topic to be removed.
  def removeSkinTopic(self, skin_topic):
    if skin_topic not in self.skin_subs.keys():
        rospy.loginfo("Skin topic not found")
        return
    with self.topic_lock:
      self.skin_topic_list.remove(skin_topic)
      self.skin_data.pop(skin_topic)
      self.skin_subs[skin_topic].unregister()
      self.skin_subs.pop(skin_topic)
  
  ## Skin Callback. Store the message in the data dictionary, indexed by topic.  
  # Keeps the raw data in dictionary 'skin_data' and the transformed, trimmed data in 'trimmed_skin_data'
  # @param msg TaxelArray message object
  # @param skin_topic The topic name triggering the callback. Used to identify what sensor the TaxelArray came from (as there may be multiple publishers running)
  def skinCallback(self, msg, skin_topic):
    with self.data_lock:
      self.skin_data[skin_topic] = msg # Data should be of type TaxelArray
      # DIRTY DIRTY DIRTY HACK to ignore pr2 wrist taxel.
#      if self.joint_angles and self.joint_angles[5] < np.radians(-90.0): 
#        #print self.joint_angles
#        # Potentially also 10 - middle forearm, 13/19 - edges
#        if skin_topic =="/pr2_fabric_forearm_sensor/taxels/forces":
#          #print "trimming value 16"
#          #print msg
#          msg.values_x = list(msg.values_x)
#          msg.values_y = list(msg.values_y)
#          msg.values_z = list(msg.values_z)
#          msg.values_x[16] = 0.0
#          msg.values_y[16] = 0.0
#          msg.values_z[16] = 0.0
#
#          #msg.values_x[10] = 0.0
#          #msg.values_y[10] = 0.0
#          #msg.values_z[10] = 0.0
#          
#          msg.values_x[13] = 0.0
#          msg.values_y[13] = 0.0
#          msg.values_z[13] = 0.0
#
#          msg.values_x[19] = 0.0
#          msg.values_y[19] = 0.0
#          msg.values_z[19] = 0.0 

      trimmed_msg = self.trimTaxelArray(msg, self.trim_threshold)
      transformed_msg = self.transformTaxelArray(trimmed_msg, self.torso_frame)
      self.trimmed_skin_data[skin_topic] = transformed_msg
      
  
  ## Transform a single taxel array message from one frame to another
  # @param ta_msg TaxelArray message object to be transformed
  # @param new_frame The desired frame name
  # @return The transformed message with all values in the new coordinate frame.
  def transformTaxelArray(self, ta_msg, new_frame):   

    # Get the transformation from the desired frame to current frame 
    if ta_msg.header.frame_id == "":
      return ta_msg
    self.tf_lstnr.waitForTransform(new_frame, ta_msg.header.frame_id, rospy.Time(0), rospy.Duration(4.0))
    t1, q1 = self.tf_lstnr.lookupTransform(new_frame, ta_msg.header.frame_id, rospy.Time(0))

    t1 = np.matrix(t1).reshape(3,1)
    r1 = tr.quaternion_to_matrix(q1)

    # Create new message data structure
    new_ta_msg = copy.copy(ta_msg)
    new_ta_msg.header.frame_id = new_frame
    

    # Perform the transformation
    pts = np.column_stack((ta_msg.centers_x, ta_msg.centers_y, ta_msg.centers_z))
    nrmls = np.column_stack((ta_msg.normals_x, ta_msg.normals_y, ta_msg.normals_z))
    values = np.column_stack((ta_msg.values_x, ta_msg.values_y, ta_msg.values_z))
    
    pts = r1 * np.matrix(pts).T + t1
    nrmls = r1 * np.matrix(nrmls).T
    values = r1 * np.matrix(values).T

    # Reformat the transformed data to be repackaged as a TaxelArray message
    pts_array = np.asarray(pts)
    nrmls_array = np.asarray(nrmls)
    values_array = np.asarray(values) 

    new_ta_msg.centers_x = pts_array[0, :].tolist()
    new_ta_msg.centers_y = pts_array[1, :].tolist()
    new_ta_msg.centers_z = pts_array[2, :].tolist()
    
    new_ta_msg.normals_x = nrmls_array[0, :].tolist()
    new_ta_msg.normals_y = nrmls_array[1, :].tolist()
    new_ta_msg.normals_z = nrmls_array[2, :].tolist()
    
    new_ta_msg.values_x = values_array[0, :].tolist()
    new_ta_msg.values_y = values_array[1, :].tolist()
    new_ta_msg.values_z = values_array[2, :].tolist()

    return new_ta_msg
    
  ## Return a trimmed copy of the the skin_data dictionary. Each TaxelArray within the structure will be trimmed.
  # @param threshold Threshold parameter (float greater than 0.0)
  def trimSkinContacts(self, threshold):
    with self.data_lock:
      skin_data = copy.copy(self.skin_data)

    for ta_topic in skin_data.keys():
      skin_data[ta_topic] = self.trimTaxelArray(skin_data[ta_topic], threshold)
  
    with self.data_lock:
      self.trimmed_skin_data = skin_data
      
    return skin_data
  
  ## Trim a passed TaxelArray to only incorporate forces of significance.
  # Returns a trimmed TaxelArray message object with forces of magnitude > threshold. The data is otherwise unchanged.
  def trimTaxelArray(self, ta_msg, threshold):
    if threshold < 0.0:
      rospy.logerr("SkinClient Error: Threshold passed to trimContacts must be >= 0.0")
      return ta_msg

    # Copy the message info data
    new_ta_msg = haptic_msgs.TaxelArray()
    new_ta_msg.header = copy.copy(ta_msg.header)
    new_ta_msg.sensor_type = copy.copy(ta_msg.sensor_type)
 
    # For each taxel entry in the TaxelArray, check if the force (or distance) is greater than the threshold
    for i in range(0, len(ta_msg.centers_x)):
      magnitude = np.sqrt(ta_msg.values_x[i]**2 + ta_msg.values_y[i]**2 + ta_msg.values_z[i]**2)
      
      threshold_valid = False
      if ta_msg.sensor_type == "force" and (magnitude >= threshold or magnitude <= -threshold):
        threshold_valid = True
      elif ta_msg.sensor_type == "distance" and (magnitude <= abs(threshold)):
        threshold_valid = True
      elif ta_msg.sensor_type == '' and (magnitude >= threshold or magnitude <= -threshold): # If nothing is set, treat it as force
        threshold_valid = True
      
      if threshold_valid:
        # Copy the values to the new data structure        
        new_ta_msg.values_x.append(ta_msg.values_x[i])
        new_ta_msg.values_y.append(ta_msg.values_y[i])
        new_ta_msg.values_z.append(ta_msg.values_z[i])
        
        new_ta_msg.centers_x.append(ta_msg.centers_x[i])
        new_ta_msg.centers_y.append(ta_msg.centers_y[i])
        new_ta_msg.centers_z.append(ta_msg.centers_z[i])
        
        new_ta_msg.normals_x.append(ta_msg.normals_x[i])
        new_ta_msg.normals_y.append(ta_msg.normals_y[i])
        new_ta_msg.normals_z.append(ta_msg.normals_z[i])
        
        # TODO SURVY: Persist the id of this taxel too.
       
        if i < len(ta_msg.link_names): # Some taxel arrays weren't publishing a link name list. Check if this exists.
          new_ta_msg.link_names.append(ta_msg.link_names[i])

    return new_ta_msg
   
  ## getSkinData accessor function 
  # Returns a copy of the skin_data dictionary
  def getSkinData(self):
    with self.data_lock:
      return copy.copy(self.skin_data)
  
  ## getTrimmedSkinData accessor function
  # Returns a copy of the trimmed_skin_data dictionary
  def getTrimmedSkinData(self):
    with self.data_lock:
      return copy.copy(self.trimmed_skin_data)

  # Returns a list of Point objects, each of which is corresponds to a taxel relative to the arm's base link frame.
  # @param ta_msg TaxelArray message type
  # @return 
  # TODO REMOVE THIS - unused?
  def getContactLocationsFromTaxelArray(self, ta_msg):    
    points_list = []
    for i in range(0, len(ta_msg.centers_x)):
      point_vector = np.matrix([ta_msg.centers_x[i], ta_msg.centers_y[i], ta_msg.centers_z[i]]).T
      points_list.append(point_vector)
    return points_list

  ## Returns a list of taxel locations and list of joint numbers after which the
  # joint torque will have no effect on the contact force, and optionally a time stamp
  # Must be implemented by every robot specific skin client.
  def getTaxelLocationAndJointList(self):
    raise RuntimeError('Unimplemented function.')
