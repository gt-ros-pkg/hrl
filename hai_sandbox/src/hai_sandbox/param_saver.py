import roslib; roslib.load_manifest('hai_sandbox')
import rospy
rospy.init_node('param_saver')
import hrl_lib.util as ut

joint_groups = {}
joint_groups['rarm']      = rospy.get_param('/r_arm_controller/joints')
joint_groups['larm']      = rospy.get_param('/l_arm_controller/joints')
joint_groups['head_traj'] = rospy.get_param('/head_traj_controller/joints')
joint_groups['torso']     = rospy.get_param('/torso_controller/joints')
ut.save_pickle(joint_groups, 'link_names.pkl')
