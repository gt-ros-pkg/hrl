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

class Behaviors:
    def __init__(self):
        rospy.init_node('linear_move', anonymous=True)
        self.cman_l     = con.ControllerManager('l')
        self.reactive_l = rgr.ReactiveGrasper(self.cman_l)
        self.cman_l.start_joint_controllers()
        self.cman_l.start_gripper_controller()
        #pass
        #self.robot = pr2.PR2()
        #self.kin = pk.PR2Kinematics(self.robot.tf_listener)

    def linear_move(self, start_location, direction, distance, arm):
        if arm == 'left':
            arm_kin = self.kin.left
        else:
            arm_kin = self.kin.right

        start_pose = arm_kin.ik(start_location)
        loc = start_location[0:3, 4]
        end_location = loc + distance*direction
        end_pose = arm_kin.ik(end_location)

        self.robot.left_arm.set_pose(start_pose, 5.)             #!!!
        self.robot.left_arm.set_pose(end_pose, 5.)               #!!!

    def linear_move_reactive(self, start_loc, movement):
        strans, srot = start_loc
        etrans = strans + movement

        start_ps = cf.create_pose_stamped(strans.T.A1.tolist() + srot.T.A1.tolist())
        end_ps   = cf.create_pose_stamped(etrans.T.A1.tolist() + srot.T.A1.tolist())

        pdb.set_trace()
        left_touching, right_touching, _ = self.reactive_l.guarded_move_cartesian(start_ps, 10.0, 5.0)
        left_touching, right_touching, _ = self.reactive_l.guarded_move_cartesian(end_ps, 10.0, 5.0)


if __name__ == '__main__':
    #import pdb
    #start_location = [0.34, 0.054, 0.87] + [0.015454981255042808, -0.02674860197736427, -0.012255429236635201, 0.999447577565171]
    #direction = np.matrix([1., 0., 0.]).T
    b = Behaviors()
    start_location = (np.matrix([0.3, 0.15, 0.9]).T, np.matrix([0., 0., 0., 0.1]))
    movement       = np.matrix([.4, 0, 0.]).T
    b.linear_move_reactive(start_location, movement)

    #self.reactive_l.move_cartesian_step(start_location, blocking = 1)
    #(left_touching, right_touching, palm_touching) = self.reactive_l.guarded_move_cartesian(grasp_pose, 10.0, 5.0)
        #self.cman_r     = con.ControllerManager('r')
        #self.reactive_r = rgr.ReactiveGrasper(self.cman_r)

        #self.cman_r.start_joint_controllers()
        #self.reactive_r.start_gripper_controller()
    
        #(pos, rot) = self.cman_l.return_cartesian_pose()
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


