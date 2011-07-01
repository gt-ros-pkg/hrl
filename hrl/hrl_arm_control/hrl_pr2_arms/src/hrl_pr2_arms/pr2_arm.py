
from threading import RLock

JOINT_NAMES_LIST = ['_shoulder_pan_joint',
                    '_shoulder_lift_joint', '_upper_arm_roll_joint',
                    '_elbow_flex_joint', '_forearm_roll_joint',
                    '_wrist_flex_joint', '_wrist_roll_joint']
JOINT_STATE_INDS_R = [17, 18, 16, 20, 19, 21, 22]
JOINT_STATE_INDS_L = [29, 30, 28, 32, 31, 33, 34]



class PR2Arm(HRLArm):
    ##
    # Initializes subscribers
    # @param arm 'r' for right, 'l' for left
    def __init__(self, arm='r'):
        self.kinematics = PR2ArmKinematics(arm)
        self.lock = RLock()
        self.ep = None

        rospy.Subscriber('joint_states', JointState, self.joint_state_cb)

        self.joint_names_list = []
        for s in JOINT_NAMES_LIST:
            self.joint_names_list.append(arm + s)
        if arm == 'r':
            self.JOINT_STATE_INDS = JOINT_STATE_INDS_R
        else:
            self.JOINT_STATE_INDS = JOINT_STATE_INDS_L
        self.joint_angles = None

    ##
    # Joint angles listener callback
    def joint_state_cb(self, msg):
        with self.lock:
            self.joint_angles = [msg.position[i] for i in self.JOINT_STATE_INDS]

    ##
    # Returns the current joint angle positions
    # @param wrapped If False returns the raw encoded positions, if True returns
    #                the angles with the forearm and wrist roll in the range -pi to pi
    def get_joint_angles(self, wrapped=False):
        with self.lock:
            if self.joint_angles is None:
                rospy.logerr("[pr2_arm_base] Joint angles haven't been filled yet")
                return None
            if wrapped:
                return self.wrap_angles(self.joint_angles)
            else:
                return np.array(self.joint_angles)


class PR2ArmJointTrajectory(PR2Arm):
    def __init__(self, arm):
        PR2Arm.__init__(self, arm)
        self.joint_action_client = actionlib.SimpleActionClient(
                                       arm + '_arm_controller/joint_trajectory_action',
                                       JointTrajectoryAction)

    ##
    # Commands joint angles to a single position
    # @param q Joint angles
    # @param time 
    def set_ep(self, jep, duration, delay=0.0):
        if jep is None or len(ep) != 7:
            raise RuntimeError("set_ep value is " + str(jep))
        jtg = JointTrajectoryGoal()
        jtg.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(delay)
        jtg.trajectory.joint_names = self.joint_names_list
        jtp = JointTrajectoryPoint()
        jtp.positions = list(jep)
        jtp.time_from_start = rospy.Duration(duration)
        jtg.trajectory.points.append(jtp)
        self.joint_action_client.send_goal(jtg)
        self.ep = copy.copy(jep)


class PR2ArmJTranspose(PR2Arm):
    def __init__(self):
        pass

    def r_cart_state_cb(self, msg):
        pass

    def set_ep(self, p, rot):
        pass

class PR2ArmJInverse(PR2Arm):
    def __init__(self):
        pass

def main():
    pr2_jt_arm = PR2ArmJointTrajectory('r')
    pr2_jt_arm.set_ep([0.]*7, 15)

if __name__ == "__main__":
    main()


