import roslib; roslib.load_manifest('hai_sandbox')
import hrl_pr2_lib.pr2 as pr2
import hrl_pr2_lib.pr2_kinematics as pk
import pdb
import hrl_lib.util as ut

class Behaviors:
    def __init__(self):
        self.robot = pr2.PR2()
        self.kin = pk.PR2Kinematics(self.robot.tf_listener)

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

if __name__ == '__main__':
    import pdb
    b = Behaviors()

    left_pose = b.robot.left.pose()
    left_cart = ut.load_pickle('start_pose.pkl')
    pdb.set_trace()
    kin_sol = b.kin.left.ik(left_cart)
    b.robot.left.set_pose(kin_sol, 5.)
    #b.linear_move(left_cart)
    #left_cart = b.kin.left.fk(left_pose)
    #pdb.set_trace()
    print left_cart


#b.linear_move()
#cart_pose = kin.left.fk('torso_lift_link', 'l_wrist_roll_link', joints)
#kin.left.ik(cart_pose, 'torso_lift_link')


