import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import actionlib
import pr2_controllers_msgs.msg as pr2m
import trajectory_msgs.msg as tm
import sensor_msgs.msg as sm
import cv

class Arm:
    def __init__(self, name):
        self.joint_names = rospy.get_param('/%s/joints' % name)
        self.client = actionlib.SimpleActionClient('/%s/joint_trajectory_action' % name, pr2m.JointTrajectoryAction)
        rospy.loginfo('waiting for server')
        self.client.wait_for_server()
        self.recorded = []

class JointTrajRecordReplay:

    def __init__(self):
        self.left_arm = Arm('l_arm_controller')
        self.right_arm = Arm('r_arm_controller')
        self.names_index = None
        rospy.Subscriber("joint_states", sm.JointState, self.joint_state_cb)
        cv.NamedWindow('keyboard_input', 1)
        self.exit = False

    def rarm_goal(self, g):
        self.right_arm.client.send_goal(g)
        self.right_arm.client.wait_for_result()
        return self.right_arm.client.get_result()

    def get_joint_states(self, msg):
        if self.names_index == None:
            self.names_index = {}
            for i, n in enumerate(msg.name):
                self.names_index[n] = i
        positions = [[msg.position[self.names_index[n]] for n in names_list] for names_list in [self.right_arm.joint_names, self.left_arm.joint_names]]
        rpos = positions[0]
        lpos = positions[1]
        return lpos, rpos

    def construct_points(self, posl, tstep):
        points = [tm.JointTrajectoryPoint() for i in range(len(posl))]
        for i in range(len(posl)):
            points[i].positions = posl[i]
            points[i].velocities = [0 for j in range(7)]
        for i in range(len(posl)):
            points[i].time_from_start = rospy.Duration(i*tstep)
        return points

    def joint_state_cb(self, msg):
        k = chr(cv.WaitKey(1) & 0xff)
        if k == 'r':
            lpos, rpos = self.get_joint_states(msg)
            self.left_arm.recorded.append(lpos)
            self.right_arm.recorded.append(rpos)
            rospy.loginfo('Recorded \nr: %s \nl: %s' % (str(rpos), str(lpos)))

        elif k == chr(27):
            self.exit = True

        elif k == 'p':
            #Construct points
            lpos, rpos = self.get_joint_states(msg)
            rospy.loginfo('playing back')
            tstep = 2.0
            l = list(self.right_arm.recorded)
            l.append(self.right_arm.recorded[0])
            l.insert(0, rpos)
            points = self.construct_points(l, tstep)

            g = pr2m.JointTrajectoryGoal()
            g.trajectory.joint_names = self.right_arm.joint_names
            g.trajectory.points = points
            #g.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(len(l) * tstep)
            g.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0)
            self.right_arm.client.send_goal(g)


if __name__ == '__main__':
    try:
        rospy.init_node('traj_client')
        jtr = JointTrajRecordReplay()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()
            if jtr.exit:
                rospy.loginfo('exiting')
                break
    except rospy.ROSInterruptException:
        print 'prog interrupted'

























#    def sample_goal(self):
#        points = [tm.JointTrajectoryPoint() for i in range(3)]
#        points[0].positions = [-.21, .44, -.56, -1.03, -13.1, -.089, -10.1377]
#        points[1].positions = [-.21, .21, -.51, -1.55, -13.18, -.856, -10.1]
#        points[2].positions = [-.21, .44, -.56, -1.03, -13.1, -.089, -10.1377]
#        for i in range(3):
#            points[i].velocities = [0 for j in range(7)]
#
#        g = pr2m.JointTrajectoryGoal()
#        g.trajectory.joint_names = self.right_arm.joint_names
#        g.trajectory.points = points
#        g.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(3.)
#        g.trajectory.points[0].time_from_start = rospy.Duration(2.0/2)
#        g.trajectory.points[1].time_from_start = rospy.Duration(4.0/2)
#        g.trajectory.points[2].time_from_start = rospy.Duration(6.0/2)
#        return g
#
